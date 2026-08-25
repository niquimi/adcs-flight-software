#pragma once

#include "types.h"

struct FdirReport {
    bool force_safe = false;
};

class FdirManager {
public:
    FdirReport evaluate(const SpacecraftState& state, ModeId current) const;
};
