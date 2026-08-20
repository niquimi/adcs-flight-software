#include "sun_model.h"

#include "math/constants.h"
#include "math/vec3.h"

#include <cmath>

namespace {
constexpr double kJ2000Jd = 2451545.0;
}  // namespace

void SunModel::reset() {
}

void SunModel::update(float timestamp_s, float sun_N[3]) {
    const double jd = kEpochJd + static_cast<double>(timestamp_s) / 86400.0;
    const double T = (jd - kJ2000Jd) / 36525.0;

    const double meanLon_deg = math::wrap_deg(280.460 + 36000.770 * T);
    const double meanAnom_rad = math::wrap_deg(357.528 + 35999.050 * T) * math::kDeg2Rad;
    const double lambda_rad = (
        meanLon_deg
        + 1.915 * std::sin(meanAnom_rad)
        + 0.020 * std::sin(2.0 * meanAnom_rad)
    ) * math::kDeg2Rad;
    const double eps_rad = (23.439 - 0.013 * T) * math::kDeg2Rad;

    const double sx = std::cos(lambda_rad);
    const double sy = std::cos(eps_rad) * std::sin(lambda_rad);
    const double sz = std::sin(eps_rad) * std::sin(lambda_rad);
    const double norm = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (norm < 1.0e-12) {
        math::vec3_zero(sun_N);
        return;
    }
    sun_N[0] = static_cast<float>(sx / norm);
    sun_N[1] = static_cast<float>(sy / norm);
    sun_N[2] = static_cast<float>(sz / norm);
}
