#pragma once

#include "operation_mode.h"
#include "types.h"

class StandbyMode : public OperationMode {
public:
    ModeId id() const override;

    void enter() override;
    void exit() override;

    AttitudeCommand update(const SpacecraftState& state) override;

    static constexpr float kEnterDetumbleRateRadps = 0.015f;
};