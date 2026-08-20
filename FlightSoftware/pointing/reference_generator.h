#pragma once

#include "types.h"

struct PointingReference {
    bool valid = false;
    float attitude_error[3] = {};
    float align_cos = 0.f;
    /** Target direction in body (sun: ŝ_B). Used for diag. */
    float body_vec[3] = {};
};

class ReferenceGenerator {
public:
    virtual ~ReferenceGenerator() = default;
    virtual PointingReference compute(const SpacecraftState& state) = 0;
};
