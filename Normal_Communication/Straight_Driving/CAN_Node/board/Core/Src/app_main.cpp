#include "app_main.h"
#include "main.h"
#include "straight_node.hpp"
static_assert(KUSE_CAN_DLC_LKAS11 == 8, "LKAS11 requires an 8-byte frame");

using kuse::straight_node;
using kuse::straight_tx_kind;
using kuse::straight_rx_result;

// Watch these in the CubeIDE debugger. They are observations, not control inputs.
extern "C" {
volatile uint32_t kuse_tx_ok[3] = {};
volatile uint32_t kuse_tx_failed[3] = {};
volatile uint32_t kuse_rx_ok = 0;
volatile uint32_t kuse_rx_bad_frame = 0;
volatile uint32_t kuse_rx_bad_crc = 0;
volatile uint32_t kuse_rx_bad_counter = 0;
volatile uint32_t kuse_rx_bad_command = 0;
volatile uint32_t kuse_rx_missing = 0;
volatile uint32_t kuse_rx_last_tick = 0;
volatile uint8_t kuse_rx_last_counter = 0;
volatile int16_t kuse_output_torque = 0;
volatile uint8_t kuse_output_enable = 0;
volatile uint8_t kuse_command_timed_out = 1;
volatile uint8_t kuse_command_recovering = 0;
volatile uint32_t kuse_scenario_cycle = 0;
volatile int16_t kuse_scenario_angle_deci_deg = 0;
volatile uint32_t kuse_can_hal_error = 0;
volatile uint32_t kuse_can_bus_off_count = 0;
}

static void publish_debug(const straight_node &node, uint32_t now) {
    kuse_output_torque = node.output_torque();
    kuse_output_enable = node.output_enabled() ? 1u : 0u;
    kuse_command_timed_out = node.timed_out() ? 1u : 0u;
    kuse_command_recovering = node.recovering() ? 1u : 0u;
    kuse_rx_last_tick = node.last_valid_rx_ms();
    kuse_rx_last_counter = node.last_lkas_counter();
    kuse_rx_missing = node.missing_lkas_frames();
    kuse_scenario_cycle = node.cycle(now);
    kuse_scenario_angle_deci_deg = node.angle_deci_deg(now);
}

static bool send_status(CAN_HandleTypeDef *hcan, straight_node &node,
                        straight_tx_kind kind, uint32_t now) {
    auto frame = node.make_status(kind, now);
    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox = 0;
    const uint8_t index = static_cast<uint8_t>(kind);
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u ||
        HAL_CAN_AddTxMessage(hcan, &header, frame.data, &mailbox) != HAL_OK) {
        ++kuse_tx_failed[index];
        kuse_can_hal_error = HAL_CAN_GetError(hcan);
        return false;
    }
    node.tx_queued(kind);
    ++kuse_tx_ok[index];
    return true;
}

static void receive_pending(CAN_HandleTypeDef *hcan, straight_node &node) {
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0u) {
        CAN_RxHeaderTypeDef header{};
        uint8_t data[8] = {};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            kuse_can_hal_error = HAL_CAN_GetError(hcan);
            return;
        }
        const auto result = node.receive_lkas(
            header.StdId, header.IDE != CAN_ID_STD, header.RTR != CAN_RTR_DATA,
            static_cast<uint8_t>(header.DLC), data, HAL_GetTick());
        switch (result) {
        case straight_rx_result::accepted: ++kuse_rx_ok; break;
        case straight_rx_result::wrong_frame: ++kuse_rx_bad_frame; break;
        case straight_rx_result::bad_crc: ++kuse_rx_bad_crc; break;
        case straight_rx_result::bad_counter: ++kuse_rx_bad_counter; break;
        case straight_rx_result::bad_command: ++kuse_rx_bad_command; break;
        }
    }
}

extern "C" void app_main_run(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef filter{};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = static_cast<uint16_t>(KUSE_CAN_ID_LKAS11 << 5);
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = static_cast<uint16_t>(0x7FFu << 5);
    filter.FilterMaskIdLow = 0x0006u; // IDE/RTR must both be zero.
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK || HAL_CAN_Start(hcan) != HAL_OK) {
        kuse_can_hal_error = HAL_CAN_GetError(hcan);
        Error_Handler();
    }

    straight_node node;
    uint32_t next_tx = HAL_GetTick();
    uint32_t restart_after = 0;
    bool bus_off = false;
    for (;;) {
        const uint32_t now = HAL_GetTick();
        const uint32_t error = HAL_CAN_GetError(hcan);
        if (!bus_off && (((hcan->Instance->ESR & CAN_ESR_BOFF) != 0u) ||
                         (error & HAL_CAN_ERROR_BOF) != 0u)) {
            bus_off = true;
            ++kuse_can_bus_off_count;
            kuse_can_hal_error = error;
            node.transport_fault();
            (void)HAL_CAN_Stop(hcan);
            restart_after = now + 100u;
        }
        if (bus_off && static_cast<int32_t>(now - restart_after) >= 0) {
            if (HAL_CAN_Start(hcan) == HAL_OK) {
                bus_off = false;
                next_tx = now;
            } else {
                kuse_can_hal_error = HAL_CAN_GetError(hcan);
                restart_after = now + 100u;
            }
        }
        if (!bus_off) receive_pending(hcan, node);
        node.poll_timeout(now);
        if (!bus_off && static_cast<int32_t>(now - next_tx) >= 0) {
            send_status(hcan, node, straight_tx_kind::wheel, now);
            send_status(hcan, node, straight_tx_kind::steering, now);
            send_status(hcan, node, straight_tx_kind::driver, now);
            next_tx += KUSE_CONTROL_PERIOD_MS;
            if (static_cast<int32_t>(now - next_tx) >= 0)
                next_tx = now + KUSE_CONTROL_PERIOD_MS;
        }
        publish_debug(node, now);
        HAL_Delay(1);
    }
}
