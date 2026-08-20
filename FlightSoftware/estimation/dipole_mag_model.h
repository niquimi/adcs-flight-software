#pragma once

/** IGRF-2020 centered dipole in ECI, matching Basilisk magneticFieldCenteredDipole. */
class DipoleMagModel {
public:
    void reset();

    /** timestamp_s + r_BN_N (m, Earth-centered) → unit B_N. */
    void update(float timestamp_s, const float r_BN_N[3], float B_N[3]);

    static constexpr double kEpochJd = 2459338.82487228;
    static constexpr float kPlanetRadius_m = 6371.2f * 1000.0f;
    static constexpr float kG10_T = -30926.00f / 1.0e9f;
    static constexpr float kG11_T = -2318.00f / 1.0e9f;
    static constexpr float kH11_T = 5817.00f / 1.0e9f;
};
