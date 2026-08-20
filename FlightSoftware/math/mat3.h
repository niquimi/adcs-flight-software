#pragma once

namespace math {

inline void mat3_zero(float m[9]) {
    for (int i = 0; i < 9; i++) {
        m[i] = 0.f;
    }
}

inline void mat3_copy(float dst[9], const float src[9]) {
    for (int i = 0; i < 9; i++) {
        dst[i] = src[i];
    }
}

/** C = A B, row-major 3x3. `out` must not alias `a` or `b`. */
inline void mat3_mul(const float a[9], const float b[9], float out[9]) {
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            out[row * 3 + col] =
                a[row * 3 + 0] * b[0 * 3 + col] +
                a[row * 3 + 1] * b[1 * 3 + col] +
                a[row * 3 + 2] * b[2 * 3 + col];
        }
    }
}

/** out = M v, row-major, v_B = C_BN v_N. `out` must not alias `v`. */
inline void mat3_mul_vec(const float m[9], const float v[3], float out[3]) {
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

inline void mat3_transpose(const float m[9], float out[9]) {
    out[0] = m[0];
    out[1] = m[3];
    out[2] = m[6];
    out[3] = m[1];
    out[4] = m[4];
    out[5] = m[7];
    out[6] = m[2];
    out[7] = m[5];
    out[8] = m[8];
}

}  // namespace math
