#pragma once

#include "types.h"

/** Desired body torque -> AttitudeCommand RW fields */
class RwMapper {
public:
    AttitudeCommand mapTorque(const float torque_Nm[3]) const;
};
