#include "kuse/protocol.hpp"

namespace kuse {

uint16_t kuse_read_u16_be(const uint8_t bytes[2]) {
    return static_cast<uint16_t>(((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]));
}

int16_t kuse_read_i16_be(const uint8_t bytes[2]) {
    return static_cast<int16_t>(kuse_read_u16_be(bytes));
}

void kuse_write_u16_be(uint8_t bytes[2], uint16_t value) {
    bytes[0] = static_cast<uint8_t>((value >> 8));
    bytes[1] = static_cast<uint8_t>(value);
}

void kuse_write_i16_be(uint8_t bytes[2], int16_t value) {
    kuse_write_u16_be(bytes, static_cast<uint16_t>(value));
}

int32_t kuse_round_half_away_from_zero(double value) {
    return value >= 0.0 ? static_cast<int32_t>((value + 0.5)) : static_cast<int32_t>((value - 0.5));
}

void kuse_encode_vehicle_state(const kuse_vehicle_state_t *state,
                               uint8_t out[KUSE_UDP_VEHICLE_STATE_SIZE]) {
    out[0] = KUSE_UDP_MESSAGE_VEHICLE_STATE;
    kuse_write_u16_be(&out[1], state->sequence);
    out[3] = KUSE_UDP_VEHICLE_STATE_PAYLOAD_SIZE;
    kuse_write_u16_be(&out[4], state->wheel_speed_fl);
    kuse_write_u16_be(&out[6], state->wheel_speed_fr);
    kuse_write_i16_be(&out[8], state->steering_angle);
    kuse_write_i16_be(&out[10], state->driver_torque);
}

kuse_decode_status_t kuse_decode_vehicle_state(const uint8_t *data, size_t size,
                                                kuse_vehicle_state_t *state) {
    if (size != KUSE_UDP_VEHICLE_STATE_SIZE) return KUSE_DECODE_BAD_LENGTH;
    if (data[0] != KUSE_UDP_MESSAGE_VEHICLE_STATE) return KUSE_DECODE_BAD_MESSAGE_ID;
    if (data[3] != KUSE_UDP_VEHICLE_STATE_PAYLOAD_SIZE) return KUSE_DECODE_BAD_PAYLOAD_LENGTH;
    if (kuse_read_u16_be(&data[4]) > 30000u || kuse_read_u16_be(&data[6]) > 30000u ||
        kuse_read_i16_be(&data[10]) < KUSE_TORQUE_MIN ||
        kuse_read_i16_be(&data[10]) > KUSE_TORQUE_MAX) return KUSE_DECODE_BAD_RANGE;
    state->sequence = kuse_read_u16_be(&data[1]);
    state->wheel_speed_fl = kuse_read_u16_be(&data[4]);
    state->wheel_speed_fr = kuse_read_u16_be(&data[6]);
    state->steering_angle = kuse_read_i16_be(&data[8]);
    state->driver_torque = kuse_read_i16_be(&data[10]);
    return KUSE_DECODE_OK;
}

void kuse_encode_steering_command(const kuse_steering_command_t *command,
                                  uint8_t out[KUSE_UDP_STEERING_COMMAND_SIZE]) {
    out[0] = KUSE_UDP_MESSAGE_STEERING_COMMAND;
    kuse_write_u16_be(&out[1], command->sequence);
    out[3] = KUSE_UDP_STEERING_COMMAND_PAYLOAD_SIZE;
    kuse_write_i16_be(&out[4], command->torque_request);
    out[6] = command->enable;
    out[7] = 0;
}

kuse_decode_status_t kuse_decode_steering_command(const uint8_t *data, size_t size,
                                                   kuse_steering_command_t *command) {
    if (size != KUSE_UDP_STEERING_COMMAND_SIZE) return KUSE_DECODE_BAD_LENGTH;
    if (data[0] != KUSE_UDP_MESSAGE_STEERING_COMMAND) return KUSE_DECODE_BAD_MESSAGE_ID;
    if (data[3] != KUSE_UDP_STEERING_COMMAND_PAYLOAD_SIZE) return KUSE_DECODE_BAD_PAYLOAD_LENGTH;
    if (data[6] > 1) return KUSE_DECODE_BAD_ENABLE;
    if (data[7] != 0) return KUSE_DECODE_BAD_RESERVED;
    if (kuse_read_i16_be(&data[4]) < KUSE_TORQUE_MIN ||
        kuse_read_i16_be(&data[4]) > KUSE_TORQUE_MAX) return KUSE_DECODE_BAD_RANGE;
    command->sequence = kuse_read_u16_be(&data[1]);
    command->torque_request = kuse_read_i16_be(&data[4]);
    command->enable = data[6];
    return KUSE_DECODE_OK;
}


} // namespace kuse
