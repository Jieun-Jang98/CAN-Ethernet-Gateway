#ifndef KUSE_TYPES_H
#define KUSE_TYPES_H

#include <cstdint>

namespace kuse {

using kuse_torque_t = int16_t;
using kuse_angle_deci_deg_t = int16_t;
using kuse_speed_centi_kph_t = uint16_t;
using kuse_sequence_t = uint16_t;

static_assert(sizeof(kuse_torque_t) == 2, "kuse_torque_t must be 16-bit");
static_assert(sizeof(kuse_sequence_t) == 2, "kuse_sequence_t must be 16-bit");


struct kuse_vehicle_state_t {
    kuse_sequence_t sequence;
    kuse_speed_centi_kph_t wheel_speed_fl;
    kuse_speed_centi_kph_t wheel_speed_fr;
    kuse_angle_deci_deg_t steering_angle;
    kuse_torque_t driver_torque;
};

struct kuse_steering_command_t {
    kuse_sequence_t sequence;
    kuse_torque_t torque_request;
    uint8_t enable;
};

enum kuse_decode_status_t {
    KUSE_DECODE_OK = 0,
    KUSE_DECODE_BAD_LENGTH,
    KUSE_DECODE_BAD_MESSAGE_ID,
    KUSE_DECODE_BAD_PAYLOAD_LENGTH,
    KUSE_DECODE_BAD_ENABLE,
    KUSE_DECODE_BAD_RESERVED,
    KUSE_DECODE_BAD_RANGE
};

struct kuse_rate_result_t {
    kuse_torque_t torque;
    bool rate_clamped;
};

} // namespace kuse

#endif

