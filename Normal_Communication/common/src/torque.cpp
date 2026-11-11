#include "kuse/torque.hpp"

namespace kuse {

static kuse_torque_t clamp_torque(kuse_torque_t value, kuse_torque_t low, kuse_torque_t high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

kuse_torque_t kuse_torque_limit_for_speed(kuse_speed_centi_kph_t speed) {
    if (speed >= 2000u && speed < 3000u) return 100;
    if (speed >= 3000u && speed < 8000u) return 60;
    if (speed >= 8000u && speed <= 12000u) return 30;
    return 0;
}

bool kuse_torque_within_limit(kuse_torque_t torque, kuse_torque_t limit) {
    return torque >= static_cast<kuse_torque_t>(-limit) && torque <= limit;
}

kuse_torque_t kuse_torque_clamp_to_limit(kuse_torque_t torque, kuse_torque_t limit) {
    return clamp_torque(torque, static_cast<kuse_torque_t>(-limit), limit);
}

kuse_rate_result_t kuse_torque_rate_limit(kuse_torque_t requested,
                                          kuse_torque_t last_can_torque,
                                          kuse_torque_t torque_limit) {
    kuse_torque_t low;
    kuse_torque_t high;
    kuse_rate_result_t result;
    if (last_can_torque > 0) {
        low = static_cast<kuse_torque_t>((last_can_torque - 7));
        if (low < -3) low = -3;
        high = static_cast<kuse_torque_t>((last_can_torque + 3));
    } else {
        low = static_cast<kuse_torque_t>((last_can_torque - 3));
        high = static_cast<kuse_torque_t>((last_can_torque + 7));
        if (high > 3) high = 3;
    }
    result.torque = clamp_torque(requested, low, high);
    result.torque = kuse_torque_clamp_to_limit(result.torque, torque_limit);
    result.rate_clamped = result.torque != requested;
    return result;
}


} // namespace kuse
