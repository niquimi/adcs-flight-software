#pragma once

#include "math/constants.h"

/** Kepler two-body position from mission orbital elements. */
class OrbitPropagator {
public:
    void reset();

    /** timestamp_s from epoch t = 0 → estimated r_BN_N (m). */
    void update(float timestamp_s, float r_BN_N[3]);

    static constexpr float kMu_m3ps2 = 3.98600436e14f;
    static constexpr float kSemiMajor_m = 7000.0f * 1000.0f;
    static constexpr float kEccentricity = 0.001f;
    static constexpr float kInclination_rad = 97.9f * math::kDeg2RadF;
    static constexpr float kRaan_rad = 30.0f * math::kDeg2RadF;
    static constexpr float kArgPeriapsis_rad = 10.0f * math::kDeg2RadF;
    static constexpr float kTrueAnomaly0_rad = 80.0f * math::kDeg2RadF;
};
