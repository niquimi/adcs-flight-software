#pragma once

#include "types.h"

class ModeDirector {
public:
    static constexpr float kEnterSafeBattery = 0.25f;

    struct Input {
        ModeId current;
        float timestamp_s;
        float boot_standby_duration_s;
        float battery_level;
        float rate_radps;
        bool ref_ok;
        bool rates_settled;
    };

    ModeId selectNextMode(const Input& in) const;
};