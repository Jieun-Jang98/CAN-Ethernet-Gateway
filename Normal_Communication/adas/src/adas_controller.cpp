#include "kuse/adas_controller.hpp"

#include "kuse/constants.hpp"
#include "kuse/torque.hpp"

#include <cstdint>

namespace kuse {

AdasController::AdasController()
    : previous_torque_(0) {
}

void AdasController::reset() {
    previous_torque_ = 0;
}

kuse_steering_command_t AdasController::compute(
    const kuse_vehicle_state_t& state,
    kuse_angle_deci_deg_t target_angle,
    kuse_sequence_t output_sequence) {

    kuse_steering_command_t command{};

    command.sequence = output_sequence;
    command.torque_request = 0;
    command.enable = 0;

    /*
     * wheel_speed 값의 단위는 0.01 km/h다.
     * 5000은 50.00 km/h를 의미한다.
     */
    const uint32_t speed_sum =
        static_cast<uint32_t>(state.wheel_speed_fl) +
        static_cast<uint32_t>(state.wheel_speed_fr);

    const auto average_speed =
        static_cast<kuse_speed_centi_kph_t>(speed_sum / 2u);

    /*
     * 속도에 따른 토크 상한:
     * 20~30 km/h 미만: 100
     * 30~80 km/h 미만: 60
     * 80~120 km/h:     30
     * 그 외:           0
     */
    const kuse_torque_t torque_limit =
        kuse_torque_limit_for_speed(average_speed);

    /*
     * 조향각 단위는 0.1도다.
     * 300 = 30.0도
     * 100 = 10.0도
     * 30  = 3.0도
     */
    kuse_angle_deci_deg_t angle_limit = 0;

    if (average_speed >= 2000u &&
        average_speed < 3000u) {

        angle_limit = 300;

    } else if (average_speed >= 3000u &&
               average_speed < 8000u) {

        angle_limit = 100;

    } else if (average_speed >= 8000u &&
               average_speed <= 12000u) {

        angle_limit = 30;
    }

    const int32_t absolute_angle =
        state.steering_angle < 0
            ? -static_cast<int32_t>(state.steering_angle)
            : static_cast<int32_t>(state.steering_angle);

    /*
     * 속도가 작동 범위를 벗어나거나,
     * 현재 조향각이 허용 범위를 벗어나면 즉시 해제한다.
     */
    if (torque_limit == 0 ||
        absolute_angle > static_cast<int32_t>(angle_limit)) {

        previous_torque_ = 0;
        return command;
    }

    /*
     * target_angle과 steering_angle은 모두 0.1도 단위다.
     *
     * 문서의 제어식:
     * torque_desired = 10 * error(degree)
     *
     * 0.1도 정수 단위에서는 두 값의 차이가 그대로
     * torque_desired가 된다.
     */
    int32_t desired =
        static_cast<int32_t>(target_angle) -
        static_cast<int32_t>(state.steering_angle);

    /*
     * CAN 토크 표현 가능 범위로 제한한다.
     */
    if (desired < KUSE_TORQUE_MIN) {
        desired = KUSE_TORQUE_MIN;
    } else if (desired > KUSE_TORQUE_MAX) {
        desired = KUSE_TORQUE_MAX;
    }

    /*
     * 직전 ADAS 송신 토크를 기준으로 변화량을 제한한다.
     *
     * 0에서 멀어지는 방향: 프레임당 최대 3
     * 0으로 돌아오는 방향: 프레임당 최대 7
     */
    const kuse_rate_result_t result =
        kuse_torque_rate_limit(
            static_cast<kuse_torque_t>(desired),
            previous_torque_,
            torque_limit);

    command.torque_request = result.torque;
    command.enable = 1;

    /*
     * 다음 20ms 계산에서 사용할 직전 송신 토크를 저장한다.
     */
    previous_torque_ = result.torque;

    return command;
}

} // namespace kuse
