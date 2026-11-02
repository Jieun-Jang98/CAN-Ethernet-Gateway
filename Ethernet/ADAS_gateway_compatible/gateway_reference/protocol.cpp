#include "protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gateway {
namespace {

std::int64_t sign_extend(std::uint64_t value, unsigned width) {
    const std::uint64_t sign = std::uint64_t{1} << (width - 1U);
    return static_cast<std::int64_t>((value ^ sign) - sign);
}

void put_u16_le(std::uint8_t* destination, std::uint16_t value) {
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

std::uint16_t get_u16_le(const std::uint8_t* source) {
    return static_cast<std::uint16_t>(source[0]) |
           (static_cast<std::uint16_t>(source[1]) << 8U);
}

void put_i16_le(std::uint8_t* destination, std::int16_t value) {
    put_u16_le(destination, static_cast<std::uint16_t>(value));
}

std::int16_t get_i16_le(const std::uint8_t* source) {
    return static_cast<std::int16_t>(get_u16_le(source));
}

template <typename T>
T clamp_round(double value, T minimum, T maximum) {
    const double bounded = std::clamp(value, static_cast<double>(minimum),
                                      static_cast<double>(maximum));
    return static_cast<T>(std::lround(bounded));
}

}  // namespace

std::uint64_t extract_intel(const std::uint8_t* data, unsigned start_bit,
                            unsigned bit_length) {
    std::uint64_t result = 0;
    for (unsigned bit = 0; bit < bit_length; ++bit) {
        const unsigned absolute_bit = start_bit + bit;
        const std::uint8_t value =
            static_cast<std::uint8_t>((data[absolute_bit / 8U] >>
                                       (absolute_bit % 8U)) &
                                      1U);
        result |= static_cast<std::uint64_t>(value) << bit;
    }
    return result;
}

void insert_intel(std::uint8_t* data, unsigned start_bit, unsigned bit_length,
                  std::uint64_t value) {
    for (unsigned bit = 0; bit < bit_length; ++bit) {
        const unsigned absolute_bit = start_bit + bit;
        const std::uint8_t mask =
            static_cast<std::uint8_t>(1U << (absolute_bit % 8U));
        if (((value >> bit) & 1U) != 0U) {
            data[absolute_bit / 8U] |= mask;
        } else {
            data[absolute_bit / 8U] &= static_cast<std::uint8_t>(~mask);
        }
    }
}

bool decode_wheel_speed(const std::uint8_t* data, std::size_t size,
                        VehicleState& state) {
    if (size < 4) {
        return false;
    }
    state.wheel_speed_fl_kph =
        static_cast<double>(extract_intel(data, 0, 14)) * 0.03125;
    state.wheel_speed_fr_kph =
        static_cast<double>(extract_intel(data, 16, 14)) * 0.03125;
    return true;
}

bool decode_steering_angle(const std::uint8_t* data, std::size_t size,
                           VehicleState& state) {
    if (size < 2) {
        return false;
    }
    const auto raw = sign_extend(extract_intel(data, 0, 16), 16);
    state.steering_angle_deg = static_cast<double>(raw) * 0.1;
    return true;
}

bool decode_steering_torque(const std::uint8_t* data, std::size_t size,
                            VehicleState& state) {
    if (size < 2) {
        return false;
    }
    const auto raw = static_cast<std::int32_t>(extract_intel(data, 0, 11));
    state.steering_column_torque = static_cast<double>(raw - 1024);
    return true;
}

std::array<std::uint8_t, 12> encode_vehicle_state_udp(
    const VehicleState& state, std::uint16_t sequence) {
    std::array<std::uint8_t, 12> packet{};
    packet[0] = kUdpVehicleState;
    put_u16_le(packet.data() + 1, sequence);
    packet[3] = 8;
    put_u16_le(packet.data() + 4,
               clamp_round<std::uint16_t>(state.wheel_speed_fl_kph * 100.0,
                                          0, 65535));
    put_u16_le(packet.data() + 6,
               clamp_round<std::uint16_t>(state.wheel_speed_fr_kph * 100.0,
                                          0, 65535));
    put_i16_le(packet.data() + 8,
               clamp_round<std::int16_t>(state.steering_angle_deg * 10.0,
                                         -32768, 32767));
    put_i16_le(packet.data() + 10,
               clamp_round<std::int16_t>(state.steering_column_torque,
                                         -32768, 32767));
    return packet;
}

std::optional<SteeringCommand> decode_steering_command_udp(
    const std::uint8_t* data, std::size_t size, std::string& error) {
    if (size != 10 || data[0] != kUdpSteeringCommand || data[3] != 6) {
        error = "steering packet must be 10 bytes: type=0x20, payload_length=6";
        return std::nullopt;
    }
    SteeringCommand command;
    command.sequence = get_u16_le(data + 1);
    command.target_angle_deg = static_cast<double>(get_i16_le(data + 4)) * 0.1;
    command.torque_limit = get_u16_le(data + 6);
    command.enabled = data[8] != 0;
    command.reason = data[9];
    if (command.torque_limit > 200) {
        error = "steering torque limit exceeds the configured maximum of 200";
        return std::nullopt;
    }
    if (command.reason > 15) {
        error = "steering reason must fit in 4 bits";
        return std::nullopt;
    }
    return command;
}

std::optional<SpeedCommand> decode_speed_command_udp(
    const std::uint8_t* data, std::size_t size, std::string& error) {
    if (size != 10 || data[0] != kUdpSpeedCommand || data[3] != 6) {
        error = "speed packet must be 10 bytes: type=0x21, payload_length=6";
        return std::nullopt;
    }
    SpeedCommand command;
    command.sequence = get_u16_le(data + 1);
    command.target_speed_kph = static_cast<double>(get_u16_le(data + 4)) * 0.01;
    command.deceleration_percent = data[6];
    command.enabled = data[7] != 0;
    command.reason = data[8];
    if (data[9] != 0) {
        error = "speed packet reserved byte must be zero";
        return std::nullopt;
    }
    if (command.target_speed_kph > 511.96875) {
        error = "target speed exceeds the 14-bit DBC-style range";
        return std::nullopt;
    }
    if (command.deceleration_percent > 100) {
        error = "deceleration percentage exceeds 100";
        return std::nullopt;
    }
    if (command.reason > 15) {
        error = "speed reason must fit in 4 bits";
        return std::nullopt;
    }
    return command;
}

std::array<std::uint8_t, 8> encode_steering_command_can(
    const SteeringCommand& command, std::uint8_t alive_counter) {
    std::array<std::uint8_t, 8> frame{};
    const auto angle_raw = clamp_round<std::int16_t>(
        command.target_angle_deg * 10.0, -32768, 32767);
    insert_intel(frame.data(), 0, 16, static_cast<std::uint16_t>(angle_raw));
    insert_intel(frame.data(), 16, 11,
                 std::min<std::uint16_t>(command.torque_limit, 200));
    insert_intel(frame.data(), 27, 1, command.enabled ? 1 : 0);
    insert_intel(frame.data(), 28, 4, command.reason & 0x0FU);
    insert_intel(frame.data(), 36, 4, alive_counter & 0x0FU);
    // Bits 48..55 are the checksum byte. Security/checksum is intentionally 0.
    return frame;
}

std::array<std::uint8_t, 8> encode_speed_command_can(
    const SpeedCommand& command, std::uint8_t alive_counter) {
    std::array<std::uint8_t, 8> frame{};
    const auto speed_raw = clamp_round<std::uint16_t>(
        command.target_speed_kph / 0.03125, 0, 16383);
    insert_intel(frame.data(), 0, 14, speed_raw);
    insert_intel(frame.data(), 16, 8, command.deceleration_percent);
    insert_intel(frame.data(), 24, 1, command.enabled ? 1 : 0);
    insert_intel(frame.data(), 25, 4, command.reason & 0x0FU);
    insert_intel(frame.data(), 32, 4, alive_counter & 0x0FU);
    // Bits 48..55 are the checksum byte. Security/checksum is intentionally 0.
    return frame;
}

}  // namespace gateway


