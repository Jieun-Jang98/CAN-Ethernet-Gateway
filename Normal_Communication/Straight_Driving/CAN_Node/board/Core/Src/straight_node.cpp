#include "straight_node.hpp"
#include "kuse/can_codec.hpp"

namespace kuse {

static_assert(KUSE_CONTROL_PERIOD_MS == 20, "STM32 period must match common protocol");
static_assert(KUSE_COMMAND_TIMEOUT_MS == 100, "STM32 timeout must match common protocol");
static_assert(KUSE_RECOVERY_VALID_COUNT == 3, "STM32 recovery count must match common protocol");

straight_node::straight_node()
    : wheel_counter_(0), steering_counter_(0), driver_counter_(0),
      lkas_order_{}, scenario_start_ms_(0), last_valid_rx_ms_(0),
      missing_lkas_frames_(0), output_torque_(0), last_lkas_counter_(0),
      zero_recovery_count_(0), scenario_started_(false), have_valid_rx_(false),
      output_enabled_(false), timed_out_(true), recovering_(false),
      release_until_next_(false) {
    kuse_order_tracker_reset(&lkas_order_, 16);
}

uint32_t straight_node::cycle(uint32_t now_ms) const {
    if (!scenario_started_) return 0;
    return (now_ms - scenario_start_ms_) / 12000u;
}

int16_t straight_node::angle_deci_deg(uint32_t now_ms) const {
    if (!scenario_started_) return 0;
    const uint32_t phase = (now_ms - scenario_start_ms_) % 12000u;
    if (phase < 2000u) return 0;
    if (phase < 4000u) return 20;
    if (phase < 6000u) return 10;
    if (phase < 8000u) return 0;
    if (phase < 10000u) return -20;
    return 0;
}

straight_can_frame straight_node::make_status(straight_tx_kind kind, uint32_t now_ms) const {
    straight_can_frame frame{};
    switch (kind) {
    case straight_tx_kind::wheel:
        frame.id = KUSE_CAN_ID_WHL_SPD11;
        frame.dlc = KUSE_CAN_DLC_WHL_SPD11;
        // 50 / 0.03125 = 1600 raw, for all four wheels.
        kuse_can_set_unsigned_le(frame.data, 0, 14, 1600);
        kuse_can_set_unsigned_le(frame.data, 16, 14, 1600);
        kuse_can_set_unsigned_le(frame.data, 32, 14, 1600);
        kuse_can_set_unsigned_le(frame.data, 48, 14, 1600);
        kuse_whl_spd11_set_counter_checksum(frame.data, wheel_counter_);
        break;
    case straight_tx_kind::steering:
        frame.id = KUSE_CAN_ID_SAS11;
        frame.dlc = KUSE_CAN_DLC_SAS11;
        kuse_encode_sas_angle_deci_deg(frame.data, angle_deci_deg(now_ms));
        kuse_can_set_unsigned_le(frame.data, 32, 4, steering_counter_);
        break;
    case straight_tx_kind::driver:
        frame.id = KUSE_CAN_ID_MDPS12;
        frame.dlc = KUSE_CAN_DLC_MDPS12;
        kuse_encode_driver_torque(frame.data, 0);
        kuse_can_set_unsigned_le(frame.data, 16, 8, driver_counter_);
        break;
    }
    return frame;
}

void straight_node::tx_queued(straight_tx_kind kind) {
    switch (kind) {
    case straight_tx_kind::wheel:
        wheel_counter_ = static_cast<uint8_t>((wheel_counter_ + 1u) & 0x0Fu);
        break;
    case straight_tx_kind::steering:
        steering_counter_ = static_cast<uint8_t>((steering_counter_ + 1u) & 0x0Fu);
        break;
    case straight_tx_kind::driver:
        driver_counter_ = static_cast<uint8_t>(driver_counter_ + 1u);
        break;
    }
}

straight_rx_result straight_node::receive_lkas(uint32_t id, bool extended, bool remote,
                                               uint8_t dlc, const uint8_t data[8],
                                               uint32_t now_ms) {
    if (id != KUSE_CAN_ID_LKAS11) return straight_rx_result::wrong_frame;
    if (extended || remote || dlc != KUSE_CAN_DLC_LKAS11) {
        if (recovering_) zero_recovery_count_ = 0;
        return straight_rx_result::wrong_frame;
    }
    if (!kuse_lkas11_crc_valid(data)) {
        if (recovering_) zero_recovery_count_ = 0;
        return straight_rx_result::bad_crc;
    }
    const int16_t torque = kuse_decode_lkas_torque(data);
    const bool enabled = kuse_decode_lkas_enable(data);
    if (!enabled && torque != 0) {
        if (recovering_) zero_recovery_count_ = 0;
        return straight_rx_result::bad_command;
    }
    const uint8_t counter = static_cast<uint8_t>(kuse_can_get_unsigned_le(data, 36, 4));
    uint16_t missing = 0;
    const kuse_order_status_t order = kuse_order_tracker_accept(&lkas_order_, counter, &missing);
    if (order == KUSE_ORDER_DUPLICATE || order == KUSE_ORDER_OLD) {
        if (recovering_) zero_recovery_count_ = 0;
        return straight_rx_result::bad_counter;
    }

    missing_lkas_frames_ += missing;
    last_lkas_counter_ = counter;
    last_valid_rx_ms_ = now_ms;
    have_valid_rx_ = true;
    timed_out_ = false;
    if (enabled && !scenario_started_) {
        scenario_started_ = true;
        scenario_start_ms_ = now_ms;
    }

    if (recovering_) {
        if (torque == 0 && (order == KUSE_ORDER_NEXT ||
                            (order == KUSE_ORDER_FIRST && zero_recovery_count_ == 0))) {
            ++zero_recovery_count_;
            if (zero_recovery_count_ >= KUSE_RECOVERY_VALID_COUNT) {
                recovering_ = false;
                release_until_next_ = true;
            }
        } else {
            zero_recovery_count_ = 0;
        }
        output_torque_ = 0;
        output_enabled_ = false;
        return straight_rx_result::accepted;
    }
    if (release_until_next_) release_until_next_ = false;
    output_torque_ = enabled ? torque : 0;
    output_enabled_ = enabled;
    return straight_rx_result::accepted;
}

void straight_node::poll_timeout(uint32_t now_ms) {
    if (!have_valid_rx_ || now_ms - last_valid_rx_ms_ < KUSE_COMMAND_TIMEOUT_MS)
        return;
    if (!timed_out_) {
        transport_fault();
    }
}

void straight_node::transport_fault() {
    timed_out_ = true;
    recovering_ = true;
    release_until_next_ = false;
    zero_recovery_count_ = 0;
    output_torque_ = 0;
    output_enabled_ = false;
    kuse_order_tracker_reset(&lkas_order_, 16);
}

} // namespace kuse
