#include "sun_reference.h"

#include "math/vec3.h"

PointingReference SunReference::compute(const SpacecraftState& state) {
    PointingReference ref;
    if (!state.css_valid) {
        ref.valid = false;
        return ref;
    }

    const float* s_B = state.css;
    ref.attitude_error[0] = -s_B[1];
    ref.attitude_error[1] = s_B[0];
    ref.attitude_error[2] = 0.f;
    ref.align_cos = s_B[2];
    math::vec3_copy(ref.body_vec, s_B);
    ref.valid = true;
    return ref;
}
