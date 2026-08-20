#include "triad.h"

#include "math/mat3.h"
#include "math/vec3.h"

namespace {
void triadFrame(const float t1[3], const float t2[3], float n1[3], float n2[3], float n3[3]) {
    math::vec3_copy(n1, t1);
    math::vec3_cross(t1, t2, n2);
    const float n2n = math::vec3_norm(n2);
    n2[0] /= n2n;
    n2[1] /= n2n;
    n2[2] /= n2n;
    math::vec3_cross(n1, n2, n3);
}
}  // namespace

void Triad::reset() {
}

bool Triad::update(
    const float sun_B[3],
    const float mag_B[3],
    const float sun_N[3],
    const float B_N[3],
    float C_BN[9]
) {
    float s_B[3];
    float b_B[3];
    float s_N[3];
    float b_N[3];
    if (!math::vec3_normalize(sun_B, s_B, kMinCssNorm) ||
        !math::vec3_normalize(mag_B, b_B, kMinMag_nT) ||
        !math::vec3_normalize(sun_N, s_N, kMinCssNorm) ||
        !math::vec3_normalize(B_N, b_N, kMinCssNorm)) {
        math::mat3_zero(C_BN);
        return false;
    }

    float cross_B[3];
    float cross_N[3];
    math::vec3_cross(s_B, b_B, cross_B);
    math::vec3_cross(s_N, b_N, cross_N);
    const float cB = math::vec3_norm(cross_B);
    const float cN = math::vec3_norm(cross_N);
    if (cB < kMinCross || cN < kMinCross) {
        math::mat3_zero(C_BN);
        return false;
    }

    float n1B[3];
    float n2B[3];
    float n3B[3];
    float n1N[3];
    float n2N[3];
    float n3N[3];
    triadFrame(s_B, b_B, n1B, n2B, n3B);
    triadFrame(s_N, b_N, n1N, n2N, n3N);

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            C_BN[row * 3 + col] =
                n1B[row] * n1N[col] + n2B[row] * n2N[col] + n3B[row] * n3N[col];
        }
    }
    return true;
}
