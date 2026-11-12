#ifndef KUSE_STRAIGHT_NODE_HPP
#define KUSE_STRAIGHT_NODE_HPP

#include <cstdint>
#include "kuse/checks.hpp"
#include "kuse/constants.hpp"

namespace kuse {

enum class straight_tx_kind : uint8_t { wheel, steering, driver };
enum class straight_rx_result : uint8_t {
    accepted, wrong_frame, bad_crc, bad_counter, bad_command
};

struct straight_can_frame {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
};

class straight_node {
public:
    straight_node();
    straight_can_frame make_status(straight_tx_kind kind, uint32_t now_ms) const;
    void tx_queued(straight_tx_kind kind);
    straight_rx_result receive_lkas(uint32_t id, bool extended, bool remote,
                                    uint8_t dlc, const uint8_t data[8], uint32_t now_ms);
    void poll_timeout(uint32_t now_ms);
    void transport_fault();

    int16_t output_torque() const { return output_torque_; }
    bool output_enabled() const { return output_enabled_; }
    bool timed_out() const { return timed_out_; }
    bool recovering() const { return recovering_; }
    bool scenario_started() const { return scenario_started_; }
    uint32_t cycle(uint32_t now_ms) const;
    int16_t angle_deci_deg(uint32_t now_ms) const;
    uint8_t last_lkas_counter() const { return last_lkas_counter_; }
    uint32_t last_valid_rx_ms() const { return last_valid_rx_ms_; }
    uint32_t missing_lkas_frames() const { return missing_lkas_frames_; }

private:
    uint8_t wheel_counter_;
    uint8_t steering_counter_;
    uint8_t driver_counter_;
    kuse_order_tracker_t lkas_order_;
    uint32_t scenario_start_ms_;
    uint32_t last_valid_rx_ms_;
    uint32_t missing_lkas_frames_;
    int16_t output_torque_;
    uint8_t last_lkas_counter_;
    uint8_t zero_recovery_count_;
    bool scenario_started_;
    bool have_valid_rx_;
    bool output_enabled_;
    bool timed_out_;
    bool recovering_;
    bool release_until_next_;
};

} // namespace kuse

#endif
