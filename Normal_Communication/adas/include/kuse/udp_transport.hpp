#ifndef KUSE_UDP_TRANSPORT_H
#define KUSE_UDP_TRANSPORT_H

#include "kuse/types.hpp"

#include <cstdint>
#include <string>

namespace kuse {

class UdpTransport {
public:
    UdpTransport(
        uint16_t local_port,
        const std::string& gateway_ip,
        uint16_t gateway_port);

    ~UdpTransport();

    UdpTransport(const UdpTransport&) = delete;
    UdpTransport& operator=(const UdpTransport&) = delete;

    bool open_socket();
    void close_socket();

    bool receive_vehicle_state(
        kuse_vehicle_state_t& state,
        uint32_t timeout_ms);

    bool send_steering_command(
        const kuse_steering_command_t& command);

private:
    int socket_fd_;
    uint16_t local_port_;
    std::string gateway_ip_;
    uint16_t gateway_port_;
};

} // namespace kuse

#endif
