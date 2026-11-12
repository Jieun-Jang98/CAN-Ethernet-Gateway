#include "kuse/adas_controller.hpp"
#include "kuse/constants.hpp"
#include "kuse/udp_transport.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

namespace {

std::atomic<bool> running{true};

void signal_handler(int) {
    running.store(false);
}

} // namespace

int main(int argc, char* argv[]) {
    constexpr uint16_t udp_port = 5000;
    constexpr uint8_t required_valid_states = 3;

    const std::string gateway_ip =
        argc >= 2 ? argv[1] : "192.168.50.11";

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    kuse::UdpTransport transport(
        udp_port,
        gateway_ip,
        udp_port);

    if (!transport.open_socket()) {
        std::fprintf(
            stderr,
            "Failed to open UDP port %u\n",
            static_cast<unsigned int>(udp_port));

        return 1;
    }

    std::printf(
        "ADAS started: local UDP port=%u, gateway=%s:%u\n",
        static_cast<unsigned int>(udp_port),
        gateway_ip.c_str(),
        static_cast<unsigned int>(udp_port));

    kuse::AdasController controller;

    kuse::kuse_sequence_t output_sequence = 0;
    uint8_t consecutive_valid_states = 0;

    while (running.load()) {
        kuse::kuse_vehicle_state_t state{};

        const bool received =
            transport.receive_vehicle_state(
                state,
                KUSE_STATE_TIMEOUT_MS);

        kuse::kuse_steering_command_t command{};
        command.sequence = output_sequence;
        command.torque_request = 0;
        command.enable = 0;

        if (!received) {
            consecutive_valid_states = 0;
            controller.reset();

            std::fprintf(
                stderr,
                "Vehicle_State timeout or invalid packet\n");
        } else {
            if (consecutive_valid_states <
                required_valid_states) {

                ++consecutive_valid_states;
            }

            if (consecutive_valid_states >=
                required_valid_states) {

                constexpr
                    kuse::kuse_angle_deci_deg_t target_angle = 0;

                command =
                    controller.compute(
                        state,
                        target_angle,
                        output_sequence);
            }

            std::printf(
                "RX seq=%u speed=%u/%u angle=%d "
                "driver=%d valid=%u -> "
                "TX seq=%u torque=%d enable=%u\n",
                static_cast<unsigned int>(state.sequence),
                static_cast<unsigned int>(state.wheel_speed_fl),
                static_cast<unsigned int>(state.wheel_speed_fr),
                static_cast<int>(state.steering_angle),
                static_cast<int>(state.driver_torque),
                static_cast<unsigned int>(
                    consecutive_valid_states),
                static_cast<unsigned int>(command.sequence),
                static_cast<int>(command.torque_request),
                static_cast<unsigned int>(command.enable));
        }

        if (transport.send_steering_command(command)) {
            ++output_sequence;
        } else {
            std::fprintf(
                stderr,
                "Failed to send Steering_Command\n");
        }
    }

    controller.reset();

    std::puts("Stopping ADAS: sending release commands");

    for (uint8_t i = 0;
         i < KUSE_STOP_RELEASE_COUNT;
         ++i) {

        kuse::kuse_steering_command_t release{};
        release.sequence = output_sequence;
        release.torque_request = 0;
        release.enable = 0;

        if (transport.send_steering_command(release)) {
            ++output_sequence;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                KUSE_STOP_RELEASE_INTERVAL_MS));
    }

    transport.close_socket();

    std::puts("ADAS stopped");
    return 0;
}
