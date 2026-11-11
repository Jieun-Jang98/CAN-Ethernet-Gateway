#include "kuse/can_codec.h"
#include "kuse/protocol.h"
#include <string.h>

uint32_t kuse_can_get_unsigned_le(const uint8_t *data, uint8_t start_bit, uint8_t bit_length) {
    uint32_t value = 0;
    uint8_t i;
    for (i = 0; i < bit_length; ++i) {
        const uint8_t bit = (uint8_t)(start_bit + i);
        value |= (uint32_t)((data[bit / 8] >> (bit % 8)) & 1u) << i;
    }
    return value;
}

int32_t kuse_can_get_signed_le(const uint8_t *data, uint8_t start_bit, uint8_t bit_length) {
    uint32_t raw = kuse_can_get_unsigned_le(data, start_bit, bit_length);
    if (bit_length < 32 && (raw & (UINT32_C(1) << (bit_length - 1))) != 0) {
        raw |= ~((UINT32_C(1) << bit_length) - 1u);
    }
    return (int32_t)raw;
}

void kuse_can_set_unsigned_le(uint8_t *data, uint8_t start_bit, uint8_t bit_length, uint32_t value) {
    uint8_t i;
    for (i = 0; i < bit_length; ++i) {
        const uint8_t bit = (uint8_t)(start_bit + i);
        const uint8_t mask = (uint8_t)(1u << (bit % 8));
        if (((value >> i) & 1u) != 0) data[bit / 8] |= mask;
        else data[bit / 8] &= (uint8_t)~mask;
    }
}

void kuse_can_set_signed_le(uint8_t *data, uint8_t start_bit, uint8_t bit_length, int32_t value) {
    kuse_can_set_unsigned_le(data, start_bit, bit_length, (uint32_t)value);
}

double kuse_decode_wheel_speed_fl(const uint8_t data[8]) {
    return (double)kuse_can_get_unsigned_le(data, 0, 14) * 0.03125;
}

double kuse_decode_wheel_speed_fr(const uint8_t data[8]) {
    return (double)kuse_can_get_unsigned_le(data, 16, 14) * 0.03125;
}

void kuse_encode_wheel_speeds(uint8_t data[8], double fl_kph, double fr_kph) {
    kuse_can_set_unsigned_le(data, 0, 14, (uint32_t)kuse_round_half_away_from_zero(fl_kph / 0.03125));
    kuse_can_set_unsigned_le(data, 16, 14, (uint32_t)kuse_round_half_away_from_zero(fr_kph / 0.03125));
}

kuse_angle_deci_deg_t kuse_decode_sas_angle_deci_deg(const uint8_t data[5]) {
    return (kuse_angle_deci_deg_t)kuse_can_get_signed_le(data, 0, 16);
}

void kuse_encode_sas_angle_deci_deg(uint8_t data[5], kuse_angle_deci_deg_t angle) {
    kuse_can_set_signed_le(data, 0, 16, angle);
}

kuse_torque_t kuse_decode_driver_torque(const uint8_t data[8]) {
    return (kuse_torque_t)((int32_t)kuse_can_get_unsigned_le(data, 0, 11) - 1024);
}

void kuse_encode_driver_torque(uint8_t data[8], kuse_torque_t torque) {
    kuse_can_set_unsigned_le(data, 0, 11, (uint32_t)((int32_t)torque + 1024));
}

kuse_torque_t kuse_decode_lkas_torque(const uint8_t data[8]) {
    return (kuse_torque_t)((int32_t)kuse_can_get_unsigned_le(data, 16, 11) - 1024);
}

bool kuse_decode_lkas_enable(const uint8_t data[8]) {
    return kuse_can_get_unsigned_le(data, 27, 1) != 0;
}

void kuse_encode_lkas_fields(uint8_t data[8], kuse_torque_t torque, bool enable,
                             uint8_t counter) {
    memset(data, 0, 8);
    kuse_can_set_unsigned_le(data, 16, 11, (uint32_t)((int32_t)torque + 1024));
    kuse_can_set_unsigned_le(data, 27, 1, enable ? 1u : 0u);
    kuse_can_set_unsigned_le(data, 36, 4, counter & 0x0Fu);
}

