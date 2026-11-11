#ifndef KUSE_PROTOCOL_H
#define KUSE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include "kuse/constants.h"
#include "kuse/types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint16_t kuse_read_u16_be(const uint8_t bytes[2]);
int16_t kuse_read_i16_be(const uint8_t bytes[2]);
void kuse_write_u16_be(uint8_t bytes[2], uint16_t value);
void kuse_write_i16_be(uint8_t bytes[2], int16_t value);

void kuse_encode_vehicle_state(const kuse_vehicle_state_t *state,
                               uint8_t out[KUSE_UDP_VEHICLE_STATE_SIZE]);
kuse_decode_status_t kuse_decode_vehicle_state(const uint8_t *data, size_t size,
                                                kuse_vehicle_state_t *state);
void kuse_encode_steering_command(const kuse_steering_command_t *command,
                                  uint8_t out[KUSE_UDP_STEERING_COMMAND_SIZE]);
kuse_decode_status_t kuse_decode_steering_command(const uint8_t *data, size_t size,
                                                   kuse_steering_command_t *command);
int32_t kuse_round_half_away_from_zero(double value);

#ifdef __cplusplus
}
#endif

#endif

