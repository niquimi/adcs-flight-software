#include "css_wls.h"

#include "math/vec3.h"

void CssWls::reset() {
}

bool CssWls::update(const float css[6], float sun_B[3]) {
    float yMax = 0.f;
    for (int i = 0; i < kNumCss; i++) {
        if (css[i] > yMax) {
            yMax = css[i];
        }
    }
    if (yMax < kMinIlluminated) {
        math::vec3_zero(sun_B);
        return false;
    }

    // One diode per body axis: keep the brighter of ±face. Do not drop a
    // small-but-real cosine (sun near a cube plane); that clipped s_B to
    // [0,0,1] and added ~2°. y = kScale * max(0, n·s).
    for (int axis = 0; axis < 3; axis++) {
        const int iPos = 2 * axis;
        const float yPos = css[iPos];
        const float yNeg = css[iPos + 1];
        if (yPos >= yNeg) {
            sun_B[axis] = yPos / kScale;
        } else {
            sun_B[axis] = -yNeg / kScale;
        }
    }

    if (!math::vec3_normalize(sun_B, sun_B, 1.0e-6f)) {
        math::vec3_zero(sun_B);
        return false;
    }
    return true;
}
