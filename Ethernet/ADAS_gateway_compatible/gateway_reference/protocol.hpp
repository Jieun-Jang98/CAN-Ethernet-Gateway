#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace gateway {

constexpr std::uint32_t kCanIdWheelSpeed = 0x386;
constexpr std::uint32_t kCanIdSteeringAngle = 0x2B0;
constexpr std::uint32_t kCanIdSteeringTorque = 0x251;
constexpr std::uint32_t kCanIdSteeringCommand = 0x340;
constexpr std::uint32_t kCanIdSpeedCommand = 0x341;

constexpr std::uint8_t kUdpVehicleState = 0x10;
constexpr std::uint8_t kUdpSteeringCommand = 0x20;
constexpr std::uint8_t kUdpSpeedCommand = 0x21;

struct VehicleState {
    double wheel_speed_fl_kph = 0.0;
    double wheel_speed_fr_kph = 0.0;
    double steering_angle_deg = 0.0;
    double steering_column_torque = 0.0;
};

struct SteeringCommand {
    std::uint16_t sequence = 0;
    double target_angle_deg = 0.0;
    std::uint16_t torque_limit = 0;
    bool enabled = false;
    std::uint8_t reason = 0;
};

struct SpeedCommand {
    std::uint16_t sequence = 0;
    double target_speed_kph = 0.0;
    std::uint8_t deceleration_percent = 0;
    bool enabled = false;
    std::uint8_t reason = 0;
};

std::uint64_t extract_intel(const std::uint8_t* data, unsigned start_bit,
                            unsigned bit_length);
void insert_intel(std::uint8_t* data, unsigned start_bit, unsigned bit_length,
                  std::uint64_t value);

bool decode_wheel_speed(const std::uint8_t* data, std::size_t size,
                        VehicleState& state);
bool decode_steering_angle(const std::uint8_t* data, std::size_t size,
                           VehicleState& state);
bool decode_steering_torque(const std::uint8_t* data, std::size_t size,
                            VehicleState& state);

std::array<std::uint8_t, 12> encode_vehicle_state_udp(
    const VehicleState& state, std::uint16_t sequence);

std::optional<SteeringCommand> decode_steering_command_udp(
    const std::uint8_t* data, std::size_t size, std::string& error);
std::optional<SpeedCommand> decode_speed_command_udp(
    const std::uint8_t* data, std::size_t size, std::string& error);

std::array<std::uint8_t, 8> encode_steering_command_can(
    const SteeringCommand& command, std::uint8_t alive_counter);
std::array<std::uint8_t, 8> encode_speed_command_can(
    const SpeedCommand& command, std::uint8_t alive_counter);

}  // namespace gateway


