#include "math/attitude.h"

#include "math/constants.h"
#include "math/vec3.h"

#include <cmath>

namespace math {

void dcm_from_mrp(const float s[3], float C[9]) {
    const float s0 = s[0];
    const float s1 = s[1];
    const float s2 = s[2];
    const float d1 = s0 * s0 + s1 * s1 + s2 * s2;
    const float S = 1.f - d1;
    const float d = (1.f + d1) * (1.f + d1);
    C[0] = (4.f * (2.f * s0 * s0 - d1) + S * S) / d;
    C[1] = (8.f * s0 * s1 + 4.f * s2 * S) / d;
    C[2] = (8.f * s0 * s2 - 4.f * s1 * S) / d;
    C[3] = (8.f * s1 * s0 - 4.f * s2 * S) / d;
    C[4] = (4.f * (2.f * s1 * s1 - d1) + S * S) / d;
    C[5] = (8.f * s1 * s2 + 4.f * s0 * S) / d;
    C[6] = (8.f * s2 * s0 + 4.f * s1 * S) / d;
    C[7] = (8.f * s2 * s1 - 4.f * s0 * S) / d;
    C[8] = (4.f * (2.f * s2 * s2 - d1) + S * S) / d;
}

void dcm_reorthogonalize(float C[9]) {
    float r0[3] = {C[0], C[1], C[2]};
    if (!vec3_normalize(r0, r0, 1.0e-12f)) {
        return;
    }

    float r1[3] = {C[3], C[4], C[5]};
    const float d = vec3_dot(r1, r0);
    r1[0] -= d * r0[0];
    r1[1] -= d * r0[1];
    r1[2] -= d * r0[2];
    if (!vec3_normalize(r1, r1, 1.0e-12f)) {
        return;
    }

    float r2[3];
    vec3_cross(r0, r1, r2);

    C[0] = r0[0];
    C[1] = r0[1];
    C[2] = r0[2];
    C[3] = r1[0];
    C[4] = r1[1];
    C[5] = r1[2];
    C[6] = r2[0];
    C[7] = r2[1];
    C[8] = r2[2];
}

float dcm_geodesic_angle_deg(const float C_a[9], const float C_b[9]) {
    float trace = 0.f;
    for (int i = 0; i < 3; i++) {
        trace += vec3_dot(&C_a[i * 3], &C_b[i * 3]);
    }
    float c = 0.5f * (trace - 1.f);
    if (c > 1.f) {
        c = 1.f;
    }
    if (c < -1.f) {
        c = -1.f;
    }
    return std::acos(c) * kRad2Deg;
}

}  // namespace math
