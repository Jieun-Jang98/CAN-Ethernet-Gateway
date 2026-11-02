#include "protocol.hpp"

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kStatePeriod = std::chrono::milliseconds(20);
constexpr auto kSensorFreshness = std::chrono::milliseconds(200);
constexpr auto kCommandTimeout = std::chrono::milliseconds(100);
std::atomic_bool running{true};

struct Options {
    std::string can_interface = "can0";
    std::string local_ip = "192.168.50.11";
    std::string adas_ip = "192.168.50.12";
    std::uint16_t state_port = 5000;
    std::uint16_t command_port = 5001;
};

struct SensorTimes {
    Clock::time_point wheel{};
    Clock::time_point angle{};
    Clock::time_point torque{};
    bool wheel_seen = false;
    bool angle_seen = false;
    bool torque_seen = false;

    bool fresh(Clock::time_point now) const {
        return wheel_seen && angle_seen && torque_seen &&
               now - wheel <= kSensorFreshness &&
               now - angle <= kSensorFreshness &&
               now - torque <= kSensorFreshness;
    }
};

void signal_handler(int) { running = false; }

void usage(const char* executable) {
    std::cout << "Usage: " << executable
              << " [--can can0] [--local-ip 192.168.50.11]"
                 " [--adas-ip 192.168.50.12] [--state-port 5000]"
                 " [--command-port 5001]\n";
}

std::uint16_t parse_port(const std::string& text) {
    const unsigned long value = std::stoul(text);
    if (value == 0 || value > 65535) {
        throw std::runtime_error("invalid UDP port: " + text);
    }
    return static_cast<std::uint16_t>(value);
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help") {
            usage(argv[0]);
            std::exit(0);
        }
        if (index + 1 >= argc) {
            throw std::runtime_error("missing value after " + argument);
        }
        const std::string value = argv[++index];
        if (argument == "--can") {
            options.can_interface = value;
        } else if (argument == "--local-ip") {
            options.local_ip = value;
        } else if (argument == "--adas-ip") {
            options.adas_ip = value;
        } else if (argument == "--state-port") {
            options.state_port = parse_port(value);
        } else if (argument == "--command-port") {
            options.command_port = parse_port(value);
        } else {
            throw std::runtime_error("unknown option: " + argument);
        }
    }
    return options;
}

sockaddr_in make_ipv4_address(const std::string& ip, std::uint16_t port) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
        throw std::runtime_error("invalid IPv4 address: " + ip);
    }
    return address;
}

int open_can_socket(const std::string& interface_name) {
    const int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        throw std::runtime_error("socket(PF_CAN) failed: " +
                                 std::string(std::strerror(errno)));
    }

    ifreq request{};
    std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &request) < 0) {
        close(socket_fd);
        throw std::runtime_error("CAN interface not found: " + interface_name);
    }

    const std::array<can_filter, 3> filters{{
        {gateway::kCanIdWheelSpeed, CAN_SFF_MASK},
        {gateway::kCanIdSteeringAngle, CAN_SFF_MASK},
        {gateway::kCanIdSteeringTorque, CAN_SFF_MASK},
    }};
    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                   sizeof(filters)) < 0) {
        close(socket_fd);
        throw std::runtime_error("setting CAN filters failed");
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) <
        0) {
        close(socket_fd);
        throw std::runtime_error("binding CAN socket failed: " +
                                 std::string(std::strerror(errno)));
    }
    return socket_fd;
}

int open_udp_receiver(const sockaddr_in& local_address) {
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        throw std::runtime_error("creating UDP receiver failed");
    }
    if (bind(socket_fd, reinterpret_cast<const sockaddr*>(&local_address),
             sizeof(local_address)) < 0) {
        const std::string message = std::strerror(errno);
        close(socket_fd);
        throw std::runtime_error("binding UDP command socket failed: " + message);
    }
    return socket_fd;
}

int open_udp_sender() {
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        throw std::runtime_error("creating UDP sender failed");
    }
    return socket_fd;
}

void send_can_frame(int socket_fd, std::uint32_t id,
                    const std::array<std::uint8_t, 8>& payload) {
    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = 8;
    std::copy(payload.begin(), payload.end(), frame.data);
    const ssize_t written = write(socket_fd, &frame, sizeof(frame));
    if (written != static_cast<ssize_t>(sizeof(frame))) {
        throw std::runtime_error("CAN write failed: " +
                                 std::string(std::strerror(errno)));
    }
}

}  // namespace

int main(int argc, char** argv) {
    int can_socket = -1;
    int udp_receive_socket = -1;
    int udp_send_socket = -1;
    try {
        const Options options = parse_options(argc, argv);
        const sockaddr_in local_command_address =
            make_ipv4_address(options.local_ip, options.command_port);
        const sockaddr_in adas_state_address =
            make_ipv4_address(options.adas_ip, options.state_port);

        can_socket = open_can_socket(options.can_interface);
        udp_receive_socket = open_udp_receiver(local_command_address);
        udp_send_socket = open_udp_sender();

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        std::cout << "Gateway started: " << options.can_interface << " <-> "
                  << options.adas_ip << ':' << options.state_port
                  << ", command listen " << options.local_ip << ':'
                  << options.command_port << '\n';

        gateway::VehicleState vehicle_state;
        SensorTimes sensor_times;
        std::uint16_t state_sequence = 0;
        std::uint8_t steering_alive = 0;
        std::uint8_t speed_alive = 0;
        auto next_state_time = Clock::now() + kStatePeriod;
        auto last_valid_command = Clock::now();
        bool command_seen = false;
        bool command_timeout_reported = false;
        bool sensor_stale_reported = false;

        std::array<pollfd, 2> descriptors{{
            {can_socket, POLLIN, 0},
            {udp_receive_socket, POLLIN, 0},
        }};

        while (running) {
            const auto before_poll = Clock::now();
            const auto wait_duration = std::max(
                std::chrono::milliseconds(0),
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    next_state_time - before_poll));
            const int poll_result =
                poll(descriptors.data(), descriptors.size(),
                     static_cast<int>(wait_duration.count()));
            if (poll_result < 0 && errno != EINTR) {
                throw std::runtime_error("poll failed: " +
                                         std::string(std::strerror(errno)));
            }

            const auto now = Clock::now();
            if ((descriptors[0].revents & POLLIN) != 0) {
                can_frame frame{};
                const ssize_t received = read(can_socket, &frame, sizeof(frame));
                if (received == static_cast<ssize_t>(sizeof(frame)) &&
                    (frame.can_id & CAN_EFF_FLAG) == 0) {
                    const std::uint32_t id = frame.can_id & CAN_SFF_MASK;
                    if (id == gateway::kCanIdWheelSpeed &&
                        gateway::decode_wheel_speed(frame.data, frame.can_dlc,
                                                    vehicle_state)) {
                        sensor_times.wheel = now;
                        sensor_times.wheel_seen = true;
                    } else if (id == gateway::kCanIdSteeringAngle &&
                               gateway::decode_steering_angle(
                                   frame.data, frame.can_dlc, vehicle_state)) {
                        sensor_times.angle = now;
                        sensor_times.angle_seen = true;
                    } else if (id == gateway::kCanIdSteeringTorque &&
                               gateway::decode_steering_torque(
                                   frame.data, frame.can_dlc, vehicle_state)) {
                        sensor_times.torque = now;
                        sensor_times.torque_seen = true;
                    }
                }
            }

            if ((descriptors[1].revents & POLLIN) != 0) {
                std::array<std::uint8_t, 256> packet{};
                const ssize_t received =
                    recvfrom(udp_receive_socket, packet.data(), packet.size(), 0,
                             nullptr, nullptr);
                if (received > 0) {
                    std::string error;
                    if (packet[0] == gateway::kUdpSteeringCommand) {
                        const auto command = gateway::decode_steering_command_udp(
                            packet.data(), static_cast<std::size_t>(received), error);
                        if (command) {
                            send_can_frame(
                                can_socket, gateway::kCanIdSteeringCommand,
                                gateway::encode_steering_command_can(
                                    *command, steering_alive++ & 0x0FU));
                        }
                    } else if (packet[0] == gateway::kUdpSpeedCommand) {
                        const auto command = gateway::decode_speed_command_udp(
                            packet.data(), static_cast<std::size_t>(received), error);
                        if (command) {
                            send_can_frame(
                                can_socket, gateway::kCanIdSpeedCommand,
                                gateway::encode_speed_command_can(
                                    *command, speed_alive++ & 0x0FU));
                        }
                    } else {
                        error = "unknown UDP command type";
                    }

                    if (error.empty()) {
                        last_valid_command = now;
                        command_seen = true;
                        command_timeout_reported = false;
                    } else {
                        std::cerr << "Rejected UDP command: " << error << '\n';
                    }
                }
            }

            if (now >= next_state_time) {
                do {
                    next_state_time += kStatePeriod;
                } while (next_state_time <= now);

                if (sensor_times.fresh(now)) {
                    const auto packet = gateway::encode_vehicle_state_udp(
                        vehicle_state, state_sequence++);
                    const ssize_t sent = sendto(
                        udp_send_socket, packet.data(), packet.size(), 0,
                        reinterpret_cast<const sockaddr*>(&adas_state_address),
                        sizeof(adas_state_address));
                    if (sent != static_cast<ssize_t>(packet.size())) {
                        std::cerr << "UDP vehicle-state send failed: "
                                  << std::strerror(errno) << '\n';
                    }
                    sensor_stale_reported = false;
                } else if (!sensor_stale_reported) {
                    std::cerr << "Vehicle state withheld: one or more CAN sensors "
                                 "are missing or older than 200 ms\n";
                    sensor_stale_reported = true;
                }
            }

            if (command_seen && now - last_valid_command > kCommandTimeout &&
                !command_timeout_reported) {
                std::cerr << "ADAS command timeout (>100 ms): CAN control "
                             "forwarding remains stopped until a valid command "
                             "arrives\n";
                command_timeout_reported = true;
            }
        }

        close(udp_send_socket);
        close(udp_receive_socket);
        close(can_socket);
        std::cout << "Gateway stopped\n";
        return 0;
    } catch (const std::exception& exception) {
        if (udp_send_socket >= 0) close(udp_send_socket);
        if (udp_receive_socket >= 0) close(udp_receive_socket);
        if (can_socket >= 0) close(can_socket);
        std::cerr << "Fatal: " << exception.what() << '\n';
        return 1;
    }
}

