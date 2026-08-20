#include "rw_mapper.h"

#include <algorithm>

namespace {
    constexpr float kMaxTorquePerWheel = 0.20f; // In Nm
}

AttitudeCommand RwMapper::mapTorque(const float torque_Nm[3]) const {
    AttitudeCommand cmd;
    
    for (int i = 0; i < 3; i++) {
        // The torque is mapped negatively because the RW need to spin opposite of the desired torque 
        cmd.rw_torque_Nm[i] = std::clamp(
            -torque_Nm[i], -kMaxTorquePerWheel, kMaxTorquePerWheel
        );
    }

    cmd.apply_rw = true;
    cmd.apply_mtb = false;

    return cmd;
}