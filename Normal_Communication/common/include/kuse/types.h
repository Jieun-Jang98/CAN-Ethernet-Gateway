#ifndef KUSE_TYPES_H
#define KUSE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int16_t kuse_torque_t;
typedef int16_t kuse_angle_deci_deg_t;
typedef uint16_t kuse_speed_centi_kph_t;
typedef uint16_t kuse_sequence_t;

#if defined(__cplusplus)
static_assert(sizeof(kuse_torque_t) == 2, "kuse_torque_t must be 16-bit");
static_assert(sizeof(kuse_sequence_t) == 2, "kuse_sequence_t must be 16-bit");
#else
_Static_assert(sizeof(kuse_torque_t) == 2, "kuse_torque_t must be 16-bit");
_Static_assert(sizeof(kuse_sequence_t) == 2, "kuse_sequence_t must be 16-bit");
#endif

typedef struct kuse_vehicle_state {
    kuse_sequence_t sequence;
    kuse_speed_centi_kph_t wheel_speed_fl;
    kuse_speed_centi_kph_t wheel_speed_fr;
    kuse_angle_deci_deg_t steering_angle;
    kuse_torque_t driver_torque;
} kuse_vehicle_state_t;

typedef struct kuse_steering_command {
    kuse_sequence_t sequence;
    kuse_torque_t torque_request;
    uint8_t enable;
} kuse_steering_command_t;

typedef enum kuse_decode_status {
    KUSE_DECODE_OK = 0,
    KUSE_DECODE_BAD_LENGTH,
    KUSE_DECODE_BAD_MESSAGE_ID,
    KUSE_DECODE_BAD_PAYLOAD_LENGTH,
    KUSE_DECODE_BAD_ENABLE,
    KUSE_DECODE_BAD_RESERVED,
    KUSE_DECODE_BAD_RANGE
} kuse_decode_status_t;

typedef struct kuse_rate_result {
    kuse_torque_t torque;
    bool rate_clamped;
} kuse_rate_result_t;

#ifdef __cplusplus
}
#endif

#endif

