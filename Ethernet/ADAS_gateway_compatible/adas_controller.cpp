// Linux: g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic adas_controller.cpp -o adas_controller
#include "control.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using Socket = SOCKET;
using AddrLen = int;
constexpr Socket BadSocket = INVALID_SOCKET;
static void closeSocket(Socket s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
using Socket = int;
using AddrLen = socklen_t;
constexpr Socket BadSocket = -1;
static void closeSocket(Socket s) { close(s); }
#endif
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
volatile std::sig_atomic_t running = 1;
void stopHandler(int) { running = 0; }

int main(int argc, char** argv) {
    // Optional arguments: target km/h (default 60), gateway IPv4.
    double targetKmh = 60;
    const char* gatewayIP = argc > 2 ? argv[2] : "192.168.50.11";
    if (argc > 3) { std::cerr << "Usage: adas_controller [target_kmh] [gateway_ipv4]\n"; return 1; }
    if (argc > 1) {
        char* end = nullptr;
        targetKmh = std::strtod(argv[1], &end);
        if (end == argv[1] || *end || !std::isfinite(targetKmh) || targetKmh < 0 || targetKmh > 120) {
            std::cerr << "Target must be 0..120 km/h\n"; return 1;
        }
    }
#ifdef _WIN32
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa)) return 1;
    struct Cleanup { ~Cleanup() { WSACleanup(); } } cleanup;
#endif
    Socket sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == BadSocket) { std::cerr << "socket failed\n"; return 1; }
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(5000);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr_in gateway{};
    gateway.sin_family = AF_INET;
    gateway.sin_port = htons(5001);
    if (inet_pton(AF_INET, gatewayIP, &gateway.sin_addr) != 1 ||
        bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        std::cerr << "Invalid Gateway IP or bind failed\n";
        closeSocket(sock); return 1;
    }
    std::signal(SIGINT, stopHandler);
    std::signal(SIGTERM, stopHandler);
    adas::Controller controller;
    adas::StateTracker tracker;
    bool wasEnabled = false;
    std::uint16_t commandCounter = 0;
    const auto epoch = Clock::now();
    auto seconds = [&](Clock::time_point t) {
        return std::chrono::duration<double>(t - epoch).count();
    };
    auto nextControl = epoch;
    auto lastControl = epoch - 20ms;
    auto sendCommands = [&](const adas::Command& cmd) {
        // Both packets share a sequence number. UDP delivery is NOT atomic.
        const auto steer = adas::steeringPacket(commandCounter, cmd);
        const auto speed = adas::speedPacket(commandCounter, cmd);
        ++commandCounter;
        auto send = [&](const auto& packet) {
            const auto sent = sendto(sock, reinterpret_cast<const char*>(packet.data()),
                                     static_cast<int>(packet.size()), 0,
                                     reinterpret_cast<const sockaddr*>(&gateway), sizeof(gateway));
            return sent == static_cast<decltype(sent)>(packet.size());
        };
        // Always attempt both, even if one send fails.
        const bool a = send(steer), b = send(speed);
        if (!a || !b) std::cerr << "UDP send failed; receiver watchdog required\n";
    };
    std::cout << "Bench ADAS: state UDP :5000, commands " << gatewayIP
              << ":5001, target=" << targetKmh << " km/h\n";
    while (running) {
        fd_set readSet;
        FD_ZERO(&readSet); FD_SET(sock, &readSet);
        timeval wait{};
        wait.tv_usec = 10000; // at most 10ms between scheduler checks
#ifdef _WIN32
        int ready = select(0, &readSet, nullptr, nullptr, &wait);
#else
        int ready = select(sock + 1, &readSet, nullptr, nullptr, &wait);
#endif
        if (ready < 0) {
            if (!running) break;
#ifndef _WIN32
            if (errno == EINTR) continue;
#endif
            std::cerr << "select failed\n";
            running = 0; break;
        }
        auto now = Clock::now();
        if (ready > 0) {
            std::uint8_t data[1024];
            sockaddr_in sender{};
            AddrLen senderLen = sizeof(sender);
            const auto n = recvfrom(sock, reinterpret_cast<char*>(data), sizeof(data), 0,
                                    reinterpret_cast<sockaddr*>(&sender), &senderLen);
            if (n >= 0 && sender.sin_addr.s_addr == gateway.sin_addr.s_addr) {
                const bool wasFresh = tracker.status(seconds(now)) == adas::Reason::Active;
                if (tracker.accept(data, static_cast<std::size_t>(n), seconds(now)) && !wasFresh)
                    controller.reset();
            }
        }
        if (now < nextControl) continue;
        const double dt = std::chrono::duration<double>(now - lastControl).count();
        // Skip missed intervals instead of sending catch-up bursts.
        nextControl += ((now - nextControl) / 20ms + 1) * 20ms;
        lastControl = now;
        adas::Command cmd;
        const adas::Reason stateStatus = tracker.status(seconds(now));
        if (stateStatus != adas::Reason::Active) {
            controller.reset();
            cmd = adas::disabled(stateStatus);
        } else {
            cmd = controller.step(tracker.state(), targetKmh, wasEnabled ? dt : 0.02);
        }
        wasEnabled = cmd.enable;
        sendCommands(cmd);
        const auto& latest = tracker.state();
        std::cout << std::fixed << std::setprecision(2)
                  << "Speed=" << (latest.speedFL + latest.speedFR) / 2
                  << " Target=" << targetKmh << " Angle=" << latest.angle
                  << " TargetAngle=" << cmd.targetAngle << " TorqueLimit=" << cmd.torqueLimit
                  << " Decel=" << int(cmd.decelerationPercent) << "%"
                  << " Enable=" << cmd.enable << " Status=" << adas::reasonName(cmd.reason) << '\n';
    }
    sendCommands(adas::disabled(adas::Reason::Shutdown));
    closeSocket(sock);
    return 0;
}

