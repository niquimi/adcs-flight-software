#pragma once

/** Complementary C_BN: gyro propagate, TRIAD blend + bias in sun; freeze b_hat in eclipse. */
class AttitudeFilter {
public:
    void reset();

    bool update(
        float timestamp_s,
        const float omega_radps[3],
        const float C_triad[9],
        bool triad_valid,
        float C_BN[9]
    );

    void gyroBias(float out[3]) const;

    static constexpr float kMaxDt_s = 1.0f;

private:
    float C_BN_[9] = {};
    float last_t_s_ = 0.f;
    bool has_lock_ = false;
    bool has_t_ = false;
    float b_hat_[3] = {};
    static constexpr float kKp = 0.5f;   // 1/s
    static constexpr float kKi = 0.02f;  // 1/s^2
    static constexpr float kMaxBiasPhi_rad = 3.f * 0.017453292519943295f;
};
