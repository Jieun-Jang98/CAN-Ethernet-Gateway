#include "control.hpp"
#include "gateway_reference/protocol.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#define CHECK(x) do { if (!(x)) throw std::runtime_error("Failed line " + std::to_string(__LINE__) + ": " #x); } while (false)
bool near(double a, double b) { return std::abs(a - b) < 0.00001; }

void checkPackets(const adas::Command& cmd, std::uint16_t seq) {
    const auto steering = adas::steeringPacket(seq, cmd);
    const auto speed = adas::speedPacket(seq, cmd);
    std::string error;
    const auto a = gateway::decode_steering_command_udp(steering.data(), steering.size(), error);
    CHECK(a && error.empty());
    const auto b = gateway::decode_speed_command_udp(speed.data(), speed.size(), error);
    CHECK(b && error.empty());
    CHECK(a->sequence == seq && b->sequence == seq);
    CHECK(a->enabled == cmd.enable && b->enabled == cmd.enable);
    CHECK(a->reason == static_cast<std::uint8_t>(cmd.reason) && b->reason == a->reason);
    if (cmd.enable) {
        CHECK(std::abs(a->target_angle_deg - cmd.targetAngle) <= 0.050001);
        CHECK(a->torque_limit == cmd.torqueLimit);
        CHECK(std::abs(b->target_speed_kph - cmd.targetSpeed) <= 0.005001);
        CHECK(b->deceleration_percent == cmd.decelerationPercent);
    } else {
        CHECK(a->target_angle_deg == 0 && a->torque_limit == 0);
        CHECK(b->target_speed_kph == 0 && b->deceleration_percent == 0);
    }
    const auto canA = gateway::encode_steering_command_can(*a, 15);
    const auto canB = gateway::encode_speed_command_can(*b, 15);
    CHECK(gateway::extract_intel(canA.data(), 27, 1) == unsigned(cmd.enable));
    CHECK(gateway::extract_intel(canB.data(), 24, 1) == unsigned(cmd.enable));
    CHECK(gateway::extract_intel(canA.data(), 36, 4) == 15);
    CHECK(gateway::extract_intel(canB.data(), 32, 4) == 15);
    CHECK(gateway::extract_intel(canA.data(), 16, 11) == a->torque_limit);
    const auto angleRaw = gateway::extract_intel(canA.data(), 0, 16);
    const double angle = (angleRaw >= 32768 ? int(angleRaw) - 65536 : int(angleRaw)) / 10.0;
    CHECK(near(angle, a->target_angle_deg));
    CHECK(std::abs(gateway::extract_intel(canB.data(), 0, 14) * 0.03125 - b->target_speed_kph) <= 0.015626);
    CHECK(gateway::extract_intel(canB.data(), 16, 8) == b->deceleration_percent);
}

int main() {
    try {
        gateway::VehicleState g;
        g.wheel_speed_fl_kph = 60; g.wheel_speed_fr_kph = 60.5;
        g.steering_angle_deg = -2.5; g.steering_column_torque = -6;
        auto bytes = gateway::encode_vehicle_state_udp(g, 0x1234);
        adas::VehicleState s;
        CHECK(adas::decodeState(bytes.data(), bytes.size(), s));
        CHECK(s.counter == 0x1234 && near(s.speedFL, 60) && near(s.speedFR, 60.5));
        CHECK(near(s.angle, -2.5) && near(s.driverTorque, -6));
        CHECK(!adas::decodeState(bytes.data(), bytes.size() - 1, s));
        bytes[0] = 1; CHECK(!adas::decodeState(bytes.data(), bytes.size(), s));

        adas::Controller c;
        s = {0, 40, 40, 4, 0};
        auto cmd = c.step(s, 60, 0.02);
        CHECK(cmd.enable && cmd.targetSpeed == 60 && cmd.decelerationPercent == 0);
        CHECK(near(cmd.targetAngle, 3.8) && cmd.torqueLimit == 5);
        checkPackets(cmd, 0x1234);
        const std::array<std::uint8_t, 10> goldenSteer{{0x20, 0x34, 0x12, 6, 38, 0, 5, 0, 1, 0}};
        const std::array<std::uint8_t, 10> goldenSpeed{{0x21, 0x34, 0x12, 6, 0x70, 0x17, 0, 1, 0, 0}};
        CHECK(adas::steeringPacket(0x1234, cmd) == goldenSteer);
        CHECK(adas::speedPacket(0x1234, cmd) == goldenSpeed);
        for (int i = 0; i < 100; ++i) cmd = c.step(s, 60, 0.02);
        CHECK(near(cmd.targetAngle, 0));
        c.reset(); s.angle = -4;
        CHECK(near(c.step(s, 60, 0.02).targetAngle, -3.8));
        c.reset(); s = {0, 20, 20, 30, 0};
        c.step(s, 60, 0.02);
        s = {0, 100, 100, 0, 0};
        CHECK(std::abs(c.step(s, 60, 0.02).targetAngle) <= 3);
        c.reset(); s = {0, 75, 75, 0, 0};
        for (int i = 0; i < 100; ++i) cmd = c.step(s, 60, 0.02);
        CHECK(cmd.decelerationPercent == 30); checkPackets(cmd, 65535);
        s.speedFL = s.speedFR = 60;
        CHECK(c.step(s, 60, 0.02).decelerationPercent == 0);
        s = {0, 100, 100, 5, 0};
        cmd = c.step(s, 60, 0.02);
        CHECK(cmd.enable && near(cmd.targetAngle, 0));
        CHECK(cmd.reason == adas::Reason::AngleLimit); checkPackets(cmd, 0);
        s = {0, 60, 60, 2, 6};
        cmd = c.step(s, 60, 0.02);
        CHECK(!cmd.enable && cmd.reason == adas::Reason::DriverOverride); checkPackets(cmd, 1);
        s = {0, 60, 80, 0, 0};
        CHECK(c.step(s, 60, 0.02).reason == adas::Reason::WheelMismatch);
        s = {0, 60, 60, 0, 0};
        CHECK(!c.step(s, 121, 0.02).enable);
        CHECK(!c.step(s, std::numeric_limits<double>::quiet_NaN(), 0.02).enable);
        CHECK(!c.step(s, 60, 0).enable);
        CHECK(!c.step(s, 60, std::numeric_limits<double>::infinity()).enable);

        adas::StateTracker tracker;
        CHECK(tracker.status(0) == adas::Reason::NoState);
        g = {}; g.wheel_speed_fl_kph = g.wheel_speed_fr_kph = 60;
        bytes = gateway::encode_vehicle_state_udp(g, 65535);
        CHECK(tracker.accept(bytes.data(), bytes.size(), 0));
        CHECK(tracker.status(0.099) == adas::Reason::Active);
        CHECK(!tracker.accept(bytes.data(), bytes.size(), 0.099)); // duplicate
        CHECK(tracker.status(0.100) == adas::Reason::Timeout);
        CHECK(!tracker.accept(bytes.data(), bytes.size(), 0.2)); // still duplicate after timeout
        bytes = gateway::encode_vehicle_state_udp(g, 0);
        CHECK(tracker.accept(bytes.data(), bytes.size(), 0.21)); // uint16 wrap
        bytes = gateway::encode_vehicle_state_udp(g, 65535);
        CHECK(!tracker.accept(bytes.data(), bytes.size(), 0.22)); // backwards
        bytes = gateway::encode_vehicle_state_udp(g, 1); bytes[3] = 9;
        CHECK(!tracker.accept(bytes.data(), bytes.size(), 0.25));
        CHECK(tracker.status(0.32) == adas::Reason::Timeout);
        g.wheel_speed_fl_kph = 300;
        bytes = gateway::encode_vehicle_state_udp(g, 1);
        CHECK(!tracker.accept(bytes.data(), bytes.size(), 0.33));
        CHECK(tracker.status(0.33) == adas::Reason::InvalidValue);
        for (int reason = 0; reason <= 9; ++reason)
            checkPackets(adas::disabled(static_cast<adas::Reason>(reason)), 42);

        // All 700 samples of the existing 14-second state scenario.
        c.reset();
        for (unsigned ms = 0; ms < 14000; ms += 20) {
            s = {0, 60, 60, 0, 0};
            if (ms >= 2000 && ms < 4000) s.angle = int((ms - 2000) / 50) / 10.0;
            else if (ms >= 4000 && ms < 6000) s.angle = (40 - int((ms - 4000) / 50)) / 10.0;
            else if (ms >= 6000 && ms < 8000) s.angle = -int((ms - 6000) / 50) / 10.0;
            else if (ms >= 8000 && ms < 10000) s.angle = (-40 + int((ms - 8000) / 50)) / 10.0;
            else if (ms >= 10000 && ms < 12000) { s.speedFL = s.speedFR = 100; s.angle = 5; }
            else if (ms >= 12000) { s.angle = 2; s.driverTorque = 6; }
            cmd = c.step(s, 60, 0.02);
            CHECK(cmd.enable == (ms < 12000));
            if (ms >= 10000 && ms < 12000) {
                CHECK(cmd.reason == adas::Reason::AngleLimit);
                CHECK(near(cmd.targetAngle, 0));
            }
            checkPackets(cmd, static_cast<std::uint16_t>(ms / 20));
        }
        std::cout << "PASS: Gateway state -> ADAS, ADAS commands -> actual Gateway UDP/CAN encoders,\n"
                     "control limits, signed values, sequence wrap, stale/invalid input, 700 scenario samples\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

