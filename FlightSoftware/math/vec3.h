#pragma once

#include "math/constants.h"

#include <cmath>

namespace math {

inline void vec3_zero(float v[3]) {
    v[0] = 0.f;
    v[1] = 0.f;
    v[2] = 0.f;
}

inline void vec3_copy(float dst[3], const float src[3]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

inline void vec3_negate(const float v[3], float out[3]) {
    out[0] = -v[0];
    out[1] = -v[1];
    out[2] = -v[2];
}

inline void vec3_scale(const float v[3], float s, float out[3]) {
    out[0] = v[0] * s;
    out[1] = v[1] * s;
    out[2] = v[2] * s;
}

inline float vec3_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void vec3_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float vec3_norm(const float v[3]) {
    return std::sqrt(vec3_dot(v, v));
}

inline bool vec3_normalize(const float v[3], float out[3], float min_norm) {
    const float n = vec3_norm(v);
    if (n < min_norm) {
        return false;
    }
    out[0] = v[0] / n;
    out[1] = v[1] / n;
    out[2] = v[2] / n;
    return true;
}

inline float vec3_angle_deg(const float a[3], const float b[3]) {
    const float na = vec3_norm(a);
    const float nb = vec3_norm(b);
    if (na < 1.0e-12f || nb < 1.0e-12f) {
        return -1.f;
    }
    float c = vec3_dot(a, b) / (na * nb);
    if (c > 1.f) {
        c = 1.f;
    }
    if (c < -1.f) {
        c = -1.f;
    }
    return std::acos(c) * kRad2Deg;
}

}  // namespace math
