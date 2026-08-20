#include "dipole_mag_model.h"

#include "math/constants.h"
#include "math/vec3.h"

#include <cmath>

namespace {
constexpr double kJ2000Jd = 2451545.0;
constexpr float kMinRadius_m = 1000.0f;

double gmstRad(double timestamp_s) {
    const double jd = DipoleMagModel::kEpochJd + timestamp_s / 86400.0;
    const double d = jd - kJ2000Jd;
    return math::wrap_deg(280.46061837 + 360.98564736629 * d) * math::kDeg2Rad;
}
}  // namespace

void DipoleMagModel::reset() {
}

void DipoleMagModel::update(float timestamp_s, const float r_BN_N[3], float B_N[3]) {
    const float r = math::vec3_norm(r_BN_N);
    if (r < kMinRadius_m) {
        math::vec3_zero(B_N);
        return;
    }

    const double gst = gmstRad(static_cast<double>(timestamp_s));
    const double cos_gst = std::cos(gst);
    const double sin_gst = std::sin(gst);

    const double rP_x = cos_gst * r_BN_N[0] + sin_gst * r_BN_N[1];
    const double rP_y = -sin_gst * r_BN_N[0] + cos_gst * r_BN_N[1];
    const double rP_z = r_BN_N[2];
    const double invR = 1.0 / static_cast<double>(r);
    const double rHat_P[3] = {rP_x * invR, rP_y * invR, rP_z * invR};

    const double m[3] = {kG11_T, kH11_T, kG10_T};
    const double mDotR =
        rHat_P[0] * m[0] + rHat_P[1] * m[1] + rHat_P[2] * m[2];
    const double scale = std::pow(static_cast<double>(kPlanetRadius_m) / r, 3.0);
    const double B_P[3] = {
        scale * (3.0 * mDotR * rHat_P[0] - m[0]),
        scale * (3.0 * mDotR * rHat_P[1] - m[1]),
        scale * (3.0 * mDotR * rHat_P[2] - m[2]),
    };

    const double BNx = cos_gst * B_P[0] - sin_gst * B_P[1];
    const double BNy = sin_gst * B_P[0] + cos_gst * B_P[1];
    const double BNz = B_P[2];
    const double bNorm = std::sqrt(BNx * BNx + BNy * BNy + BNz * BNz);
    if (bNorm < 1.0e-18) {
        math::vec3_zero(B_N);
        return;
    }
    B_N[0] = static_cast<float>(BNx / bNorm);
    B_N[1] = static_cast<float>(BNy / bNorm);
    B_N[2] = static_cast<float>(BNz / bNorm);
}
