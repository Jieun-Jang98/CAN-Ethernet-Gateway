#ifndef KUSE_CAN_CODEC_H
#define KUSE_CAN_CODEC_H

#include <cstdint>
#include "kuse/types.hpp"

namespace kuse {

uint32_t kuse_can_get_unsigned_le(const uint8_t *data, uint8_t start_bit, uint8_t bit_length);
int32_t kuse_can_get_signed_le(const uint8_t *data, uint8_t start_bit, uint8_t bit_length);
void kuse_can_set_unsigned_le(uint8_t *data, uint8_t start_bit, uint8_t bit_length, uint32_t value);
void kuse_can_set_signed_le(uint8_t *data, uint8_t start_bit, uint8_t bit_length, int32_t value);

double kuse_decode_wheel_speed_fl(const uint8_t data[8]);
double kuse_decode_wheel_speed_fr(const uint8_t data[8]);
void kuse_encode_wheel_speeds(uint8_t data[8], double fl_kph, double fr_kph);
kuse_angle_deci_deg_t kuse_decode_sas_angle_deci_deg(const uint8_t data[5]);
void kuse_encode_sas_angle_deci_deg(uint8_t data[5], kuse_angle_deci_deg_t angle);
kuse_torque_t kuse_decode_driver_torque(const uint8_t data[8]);
void kuse_encode_driver_torque(uint8_t data[8], kuse_torque_t torque);
kuse_torque_t kuse_decode_lkas_torque(const uint8_t data[8]);
bool kuse_decode_lkas_enable(const uint8_t data[8]);
void kuse_encode_lkas_fields(uint8_t data[8], kuse_torque_t torque, bool enable,
                             uint8_t counter);

} // namespace kuse

#endif
