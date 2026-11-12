#include "kuse/udp_transport.hpp"

#include "kuse/constants.hpp"
#include "kuse/protocol.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace kuse {

UdpTransport::UdpTransport(
    uint16_t local_port,
    const std::string& gateway_ip,
    uint16_t gateway_port)
    : socket_fd_(-1),
      local_port_(local_port),
      gateway_ip_(gateway_ip),
      gateway_port_(gateway_port) {
}

UdpTransport::~UdpTransport() {
    close_socket();
}

bool UdpTransport::open_socket() {
    if (socket_fd_ >= 0) {
        return true;
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_fd_ < 0) {
        return false;
    }

    sockaddr_in local_address{};
    local_address.sin_family = AF_INET;
    local_address.sin_port = htons(local_port_);
    local_address.sin_addr.s_addr = htonl(INADDR_ANY);

    const int bind_result =
        ::bind(
            socket_fd_,
            reinterpret_cast<const sockaddr*>(&local_address),
            sizeof(local_address));

    if (bind_result < 0) {
        close_socket();
        return false;
    }

    return true;
}

void UdpTransport::close_socket() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool UdpTransport::receive_vehicle_state(
    kuse_vehicle_state_t& state,
    uint32_t timeout_ms) {

    if (socket_fd_ < 0) {
        return false;
    }

    pollfd descriptor{};
    descriptor.fd = socket_fd_;
    descriptor.events = POLLIN;

    int poll_result;

    do {
        poll_result =
            ::poll(
                &descriptor,
                1,
                static_cast<int>(timeout_ms));
    } while (poll_result < 0 && errno == EINTR);

    if (poll_result <= 0) {
        return false;
    }

    if ((descriptor.revents & POLLIN) == 0) {
        return false;
    }

    std::array<uint8_t, 256> receive_buffer{};

    sockaddr_in source_address{};
    socklen_t source_length = sizeof(source_address);

    const ssize_t received_size =
        ::recvfrom(
            socket_fd_,
            receive_buffer.data(),
            receive_buffer.size(),
            0,
            reinterpret_cast<sockaddr*>(&source_address),
            &source_length);

    if (received_size < 0) {
        return false;
    }

    if (source_address.sin_family != AF_INET) {
        return false;
    }

    if (ntohs(source_address.sin_port) != gateway_port_) {
        return false;
    }

    in_addr expected_gateway_address{};

    const int address_result =
        ::inet_pton(
            AF_INET,
            gateway_ip_.c_str(),
            &expected_gateway_address);

    if (address_result != 1) {
        return false;
    }

    if (source_address.sin_addr.s_addr !=
        expected_gateway_address.s_addr) {

        return false;
    }

    if (received_size != KUSE_UDP_VEHICLE_STATE_SIZE) {
        return false;
    }

    const kuse_decode_status_t decode_status =
        kuse_decode_vehicle_state(
            receive_buffer.data(),
            static_cast<std::size_t>(received_size),
            &state);

    return decode_status == KUSE_DECODE_OK;
}

bool UdpTransport::send_steering_command(
    const kuse_steering_command_t& command) {

    if (socket_fd_ < 0) {
        return false;
    }

    std::array<uint8_t, KUSE_UDP_STEERING_COMMAND_SIZE>
        send_buffer{};

    kuse_encode_steering_command(
        &command,
        send_buffer.data());

    sockaddr_in gateway_address{};
    gateway_address.sin_family = AF_INET;
    gateway_address.sin_port = htons(gateway_port_);

    const int address_result =
        ::inet_pton(
            AF_INET,
            gateway_ip_.c_str(),
            &gateway_address.sin_addr);

    if (address_result != 1) {
        return false;
    }

    const ssize_t sent_size =
        ::sendto(
            socket_fd_,
            send_buffer.data(),
            send_buffer.size(),
            0,
            reinterpret_cast<const sockaddr*>(&gateway_address),
            sizeof(gateway_address));

    return sent_size ==
        static_cast<ssize_t>(send_buffer.size());
}

} // namespace kuse
