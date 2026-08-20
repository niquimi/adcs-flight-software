#include "nadir_reference.h"

#include "math/vec3.h"

PointingReference NadirReference::compute(const SpacecraftState& state) {
    PointingReference ref;

    if (!state.nadir_valid) {
        ref.valid = false;
        return ref;
    }

    float n_B[3];
    constexpr float kMinNadirNorm = 0.05f;
    if (!math::vec3_normalize(state.nadir_B, n_B, kMinNadirNorm)) {
        ref.valid = false;
        return ref;
    }

    // Align body -Z with nadir: error = (-Z) × n_B, align_cos = (-Z) · n_B.
    ref.attitude_error[0] = n_B[1];
    ref.attitude_error[1] = -n_B[0];
    ref.attitude_error[2] = .0f;
    ref.align_cos = -n_B[2];
    math::vec3_copy(ref.body_vec, n_B);
    ref.valid = true;
    return ref;
}
