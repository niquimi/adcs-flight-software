#include "orbit_propagator.h"

#include "math/constants.h"

#include <cmath>

namespace {
constexpr int kKeplerMaxIters = 12;
constexpr float kKeplerTol = 1.0e-8f;

float trueToEccentric(float f, float e) {
    const float cos_f = std::cos(f);
    const float sin_f = std::sin(f);
    const float den = 1.f + e * cos_f;
    const float cos_E = (e + cos_f) / den;
    const float sin_E = std::sqrt(1.f - e * e) * sin_f / den;
    return std::atan2(sin_E, cos_E);
}

float meanToEccentric(float M, float e) {
    float E = M;
    for (int i = 0; i < kKeplerMaxIters; i++) {
        const float dE = (E - e * std::sin(E) - M) / (1.f - e * std::cos(E));
        E -= dE;
        if (std::fabs(dE) < kKeplerTol) {
            break;
        }
    }
    return E;
}

float eccentricToTrue(float E, float e) {
    const float cos_E = std::cos(E);
    const float sin_E = std::sin(E);
    const float den = 1.f - e * cos_E;
    const float cos_f = (cos_E - e) / den;
    const float sin_f = std::sqrt(1.f - e * e) * sin_E / den;
    return std::atan2(sin_f, cos_f);
}

void elem2rv(
    float a,
    float e,
    float i,
    float raan,
    float argp,
    float f,
    float r_BN_N[3]
) {
    const float p = a * (1.f - e * e);
    const float r = p / (1.f + e * std::cos(f));
    const float theta = argp + f;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float cos_raan = std::cos(raan);
    const float sin_raan = std::sin(raan);
    const float cos_i = std::cos(i);
    const float sin_i = std::sin(i);

    r_BN_N[0] = r * (cos_theta * cos_raan - cos_i * sin_theta * sin_raan);
    r_BN_N[1] = r * (cos_theta * sin_raan + cos_i * sin_theta * cos_raan);
    r_BN_N[2] = r * (sin_theta * sin_i);
}
}  // namespace

void OrbitPropagator::reset() {
}

void OrbitPropagator::update(float timestamp_s, float r_BN_N[3]) {
    const float E0 = trueToEccentric(kTrueAnomaly0_rad, kEccentricity);
    const float M0 = E0 - kEccentricity * std::sin(E0);
    const float n = std::sqrt(
        kMu_m3ps2 / (kSemiMajor_m * kSemiMajor_m * kSemiMajor_m)
    );
    const float M = math::wrap_two_pi(M0 + n * timestamp_s);
    const float E = meanToEccentric(M, kEccentricity);
    const float f = eccentricToTrue(E, kEccentricity);
    elem2rv(
        kSemiMajor_m,
        kEccentricity,
        kInclination_rad,
        kRaan_rad,
        kArgPeriapsis_rad,
        f,
        r_BN_N
    );
}
