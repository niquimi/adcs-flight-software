#pragma once

#include "operation_mode.h"
#include "types.h"
#include "control/pd_controller.h"
#include "actuators/rw_mapper.h"
#include "pointing/sun_reference.h"

class SafeMode : public OperationMode {
public:

    SafeMode();

    ModeId id() const override;

    void enter() override;
    void exit() override;

    AttitudeCommand update(const SpacecraftState& state) override;

    static constexpr float kExitBatteryPercentage = 0.35f;

private:
    SunReference sunReference_;
    PdController pd_;
    RwMapper rwMapper_;
};