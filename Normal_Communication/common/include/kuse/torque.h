#ifndef KUSE_TORQUE_H
#define KUSE_TORQUE_H

#include <stdbool.h>
#include "kuse/types.h"

#ifdef __cplusplus
extern "C" {
#endif

kuse_torque_t kuse_torque_limit_for_speed(kuse_speed_centi_kph_t speed);
bool kuse_torque_within_limit(kuse_torque_t torque, kuse_torque_t limit);
kuse_torque_t kuse_torque_clamp_to_limit(kuse_torque_t torque, kuse_torque_t limit);
kuse_rate_result_t kuse_torque_rate_limit(kuse_torque_t requested,
                                          kuse_torque_t last_can_torque,
                                          kuse_torque_t torque_limit);

#ifdef __cplusplus
}
#endif

#endif

