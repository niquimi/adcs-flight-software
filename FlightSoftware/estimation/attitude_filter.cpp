#include "attitude_filter.h"

#include "math/attitude.h"
#include "math/mat3.h"
#include "math/vec3.h"

namespace {
/** C⁺ = exp(-[ω×] dt) C, second-order. */
void integrateGyro(const float omega[3], float dt, float C[9]) {
    const float wx = omega[0] * dt;
    const float wy = omega[1] * dt;
    const float wz = omega[2] * dt;

    float R[9];
    R[0] = 1.f - 0.5f * (wy * wy + wz * wz);
    R[1] = wz + 0.5f * wx * wy;
    R[2] = -wy + 0.5f * wx * wz;
    R[3] = -wz + 0.5f * wy * wx;
    R[4] = 1.f - 0.5f * (wx * wx + wz * wz);
    R[5] = wx + 0.5f * wy * wz;
    R[6] = wy + 0.5f * wz * wx;
    R[7] = -wx + 0.5f * wz * wy;
    R[8] = 1.f - 0.5f * (wx * wx + wy * wy);

    float Cnew[9];
    math::mat3_mul(R, C, Cnew);
    math::mat3_copy(C, Cnew);
    math::dcm_reorthogonalize(C);
}

/** φ such that C_meas ≈ exp(-[φ×]) C_pred (same convention as integrateGyro). */
void dcmErrorPhi(const float C_meas[9], const float C_pred[9], float phi[3]) {
    float C_pred_T[9];
    math::mat3_transpose(C_pred, C_pred_T);
    float C_err[9];
    math::mat3_mul(C_meas, C_pred_T, C_err);
    phi[0] = 0.5f * (C_err[5] - C_err[7]);
    phi[1] = 0.5f * (C_err[6] - C_err[2]);
    phi[2] = 0.5f * (C_err[1] - C_err[3]);
}
}  // namespace

void AttitudeFilter::reset() {
    math::mat3_zero(C_BN_);
    last_t_s_ = 0.f;
    has_lock_ = false;
    has_t_ = false;
    math::vec3_zero(b_hat_);
}

void AttitudeFilter::gyroBias(float out[3]) const {
    math::vec3_copy(out, b_hat_);
}

bool AttitudeFilter::update(
    float timestamp_s,
    const float omega_radps[3],
    const float C_triad[9],
    bool triad_valid,
    float C_BN[9]
) {
    const bool have_dt = has_t_;
    const float dt = timestamp_s - last_t_s_;
    last_t_s_ = timestamp_s;
    has_t_ = true;
    const bool dt_ok = have_dt && dt > 0.f && dt <= kMaxDt_s;

    if (has_lock_ && dt_ok) {
        const float omega_c[3] = {
            omega_radps[0] - b_hat_[0],
            omega_radps[1] - b_hat_[1],
            omega_radps[2] - b_hat_[2],
        };
        integrateGyro(omega_c, dt, C_BN_);
    }

    if (triad_valid) {
        if (!has_lock_ || !dt_ok) {
            math::mat3_copy(C_BN_, C_triad);
            has_lock_ = true;
        } else {
            float phi[3];
            dcmErrorPhi(C_triad, C_BN_, phi);
            const float omega_corr[3] = {kKp * phi[0], kKp * phi[1], kKp * phi[2]};
            integrateGyro(omega_corr, dt, C_BN_);
            if (math::vec3_norm(phi) <= kMaxBiasPhi_rad) {
                b_hat_[0] -= kKi * phi[0] * dt;
                b_hat_[1] -= kKi * phi[1] * dt;
                b_hat_[2] -= kKi * phi[2] * dt;
            }
        }
    }

    if (!has_lock_) {
        math::mat3_zero(C_BN);
        return false;
    }
    math::mat3_copy(C_BN, C_BN_);
    return true;
}
