#pragma once

#include "types.h"

class ModeDirector {
public:
    struct Input {
        ModeId current;
        float timestamp_s;
        float boot_standby_duration_s;
        float rate_radps;
        bool ref_ok;
        bool rates_settled;
        bool force_safe;
        bool allow_exit_safe;
    };

    ModeId selectNextMode(const Input& in) const;
};