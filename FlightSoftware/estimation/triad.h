#pragma once

/** TRIAD attitude: C_BN from sun+mag in body vs inertial. Row-major, v_B = C_BN v_N. */
class Triad {
public:
    void reset();

    bool update(
        const float sun_B[3],
        const float mag_B[3],
        const float sun_N[3],
        const float B_N[3],
        float C_BN[9]
    );

    static constexpr float kMinCssNorm = 0.05f;
    static constexpr float kMinMag_nT = 500.0f;
    static constexpr float kMinCross = 0.05f;
};
