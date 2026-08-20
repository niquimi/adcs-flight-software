#pragma once

/** Low-precision sun direction in ECI/J2000 from mission epoch + timestamp. */
class SunModel {
public:
    void reset();

    /** timestamp_s from epoch t = 0 → unit sun_N. */
    void update(float timestamp_s, float sun_N[3]);

    /** Julian date of SPICE_EPOCH "2021 MAY 04 07:47:48.965 (UTC)". */
    static constexpr double kEpochJd = 2459338.82487228;
};
