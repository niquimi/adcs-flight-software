#pragma once

#include <cmath>

namespace math {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kRad2Deg = 57.2957795f;
constexpr float kDeg2RadF = 0.017453292519943295f;
constexpr double kDeg2Rad = 0.017453292519943295;

inline double wrap_deg(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg;
}

inline float wrap_two_pi(float angle) {
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.f) {
        angle += kTwoPi;
    }
    return angle;
}

}  // namespace math
