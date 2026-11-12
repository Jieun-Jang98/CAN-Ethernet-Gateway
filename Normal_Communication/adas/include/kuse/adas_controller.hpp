#ifndef KUSE_ADAS_CONTROLLER_H
#define KUSE_ADAS_CONTROLLER_H

#include "kuse/types.hpp"

namespace kuse {

class AdasController {
public:
    AdasController();

    void reset();

    kuse_steering_command_t compute(
        const kuse_vehicle_state_t& state,
        kuse_angle_deci_deg_t target_angle,
        kuse_sequence_t output_sequence);

private:
    kuse_torque_t previous_torque_;
};

} // namespace kuse

#endif
