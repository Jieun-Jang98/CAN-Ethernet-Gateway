#include "protocol.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

namespace {

bool near(double left, double right) { return std::abs(left - right) < 0.0001; }

}  // namespace

int main() {
    gateway::VehicleState state;

    std::array<std::uint8_t, 8> wheel{};
    gateway::insert_intel(wheel.data(), 0, 14, 1600);   // 50 km/h
    gateway::insert_intel(wheel.data(), 16, 14, 1616);  // 50.5 km/h
    assert(gateway::decode_wheel_speed(wheel.data(), wheel.size(), state));
    assert(near(state.wheel_speed_fl_kph, 50.0));
    assert(near(state.wheel_speed_fr_kph, 50.5));

    std::array<std::uint8_t, 8> angle{};
    gateway::insert_intel(angle.data(), 0, 16,
                          static_cast<std::uint16_t>(static_cast<std::int16_t>(-25)));
    assert(gateway::decode_steering_angle(angle.data(), angle.size(), state));
    assert(near(state.steering_angle_deg, -2.5));

    std::array<std::uint8_t, 8> torque{};
    gateway::insert_intel(torque.data(), 0, 11, 1124);  // physical +100
    assert(gateway::decode_steering_torque(torque.data(), torque.size(), state));
    assert(near(state.steering_column_torque, 100.0));

    const auto state_packet = gateway::encode_vehicle_state_udp(state, 0x1234);
    assert(state_packet[0] == 0x10);
    assert(state_packet[1] == 0x34 && state_packet[2] == 0x12);
    assert(state_packet[3] == 8);

    const std::array<std::uint8_t, 10> steering_udp{
        0x20, 1, 0, 6, 0, 0, 200, 0, 1, 2};
    std::string error;
    const auto steering = gateway::decode_steering_command_udp(
        steering_udp.data(), steering_udp.size(), error);
    assert(steering && error.empty());
    const auto steering_can =
        gateway::encode_steering_command_can(*steering, 15);
    assert(gateway::extract_intel(steering_can.data(), 16, 11) == 200);
    assert(gateway::extract_intel(steering_can.data(), 27, 1) == 1);
    assert(gateway::extract_intel(steering_can.data(), 28, 4) == 2);
    assert(gateway::extract_intel(steering_can.data(), 36, 4) == 15);

    const std::array<std::uint8_t, 10> speed_udp{
        0x21, 2, 0, 6, 0x40, 0x1F, 30, 1, 1, 0};  // 80.00 km/h
    error.clear();
    const auto speed = gateway::decode_speed_command_udp(
        speed_udp.data(), speed_udp.size(), error);
    assert(speed && error.empty());
    assert(near(speed->target_speed_kph, 80.0));
    const auto speed_can = gateway::encode_speed_command_can(*speed, 3);
    assert(gateway::extract_intel(speed_can.data(), 0, 14) == 2560);
    assert(gateway::extract_intel(speed_can.data(), 16, 8) == 30);
    assert(gateway::extract_intel(speed_can.data(), 24, 1) == 1);
    assert(gateway::extract_intel(speed_can.data(), 32, 4) == 3);

    std::cout << "All protocol tests passed\n";
    return 0;
}

