#pragma once

#include "types.h"

struct EpsReport {
    float soc = 0.f;
    bool request_safe = false;
    bool allow_exit_safe = false;
};

class EpsManager {
public:
    static constexpr float kEnterSafeSoc = 0.25f;
    static constexpr float kExitSafeSoc = 0.35f;

    EpsReport evaluate(float soc) const;

    /** Same watts as the Basilisk plant loads. Not used to limit torque yet. */
    static float modeLoadW(ModeId mode);
};