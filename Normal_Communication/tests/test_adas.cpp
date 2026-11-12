#include "kuse/adas_controller.hpp"

#include <cstdio>

using namespace kuse;

static int failures = 0;

#define CHECK(expr) do { \
    if (!(expr)) { \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", \
                     __FILE__, __LINE__, #expr); \
        ++failures; \
    } \
} while (0)

static void test_straight_assist() {
    AdasController controller;

    kuse_vehicle_state_t state{};
    state.sequence = 0;
    state.wheel_speed_fl = 5000;
    state.wheel_speed_fr = 5000;
    state.steering_angle = 20;
    state.driver_torque = 0;

    const kuse_torque_t expected[] = {
        -3, -6, -9, -12, -15, -18, -20
    };

    for (unsigned int i = 0;
         i < sizeof(expected) / sizeof(expected[0]);
         ++i) {

        const kuse_steering_command_t command =
            controller.compute(
                state,
                0,
                static_cast<kuse_sequence_t>(i));

        CHECK(command.sequence == i);
        CHECK(command.enable == 1);
        CHECK(command.torque_request == expected[i]);
    }
}

static void test_inactive_speed() {
    AdasController controller;

    kuse_vehicle_state_t state{};
    state.wheel_speed_fl = 1900;
    state.wheel_speed_fr = 1900;
    state.steering_angle = 20;
    state.driver_torque = 0;

    const kuse_steering_command_t command =
        controller.compute(state, 0, 0);

    CHECK(command.enable == 0);
    CHECK(command.torque_request == 0);
}

static void test_angle_outside_limit() {
    AdasController controller;

    kuse_vehicle_state_t state{};
    state.wheel_speed_fl = 5000;
    state.wheel_speed_fr = 5000;
    state.steering_angle = 101;
    state.driver_torque = 0;

    const kuse_steering_command_t command =
        controller.compute(state, 0, 0);

    CHECK(command.enable == 0);
    CHECK(command.torque_request == 0);
}

static void test_reset() {
    AdasController controller;

    kuse_vehicle_state_t state{};
    state.wheel_speed_fl = 5000;
    state.wheel_speed_fr = 5000;
    state.steering_angle = 20;
    state.driver_torque = 0;

    CHECK(controller.compute(state, 0, 0).torque_request == -3);
    CHECK(controller.compute(state, 0, 1).torque_request == -6);

    controller.reset();

    CHECK(controller.compute(state, 0, 2).torque_request == -3);
}

int main() {
    test_straight_assist();
    test_inactive_speed();
    test_angle_outside_limit();
    test_reset();

    if (failures != 0) {
        std::fprintf(stderr, "%d ADAS test(s) failed\n", failures);
        return 1;
    }

    std::puts("All ADAS controller tests passed.");
    return 0;
}
