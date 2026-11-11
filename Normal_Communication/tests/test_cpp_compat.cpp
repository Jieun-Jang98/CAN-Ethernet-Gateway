#include "kuse/common.h"

int main() {
    kuse_steering_command_t command{};
    command.torque_request = -3;
    return kuse_torque_within_limit(command.torque_request, 60) ? 0 : 1;
}

