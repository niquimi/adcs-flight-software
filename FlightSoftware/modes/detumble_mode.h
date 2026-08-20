#pragma once

#include "operation_mode.h"
#include "types.h"
#include "control/pd_controller.h"
#include "actuators/rw_mapper.h"

/** Detumble mode — rate damping */
class DetumbleMode : public OperationMode {
public:

    DetumbleMode();

    ModeId id() const override;

    void enter() override;
    void exit() override;

    AttitudeCommand update(const SpacecraftState& state) override;

    bool ratesSettled() const;

    static constexpr float kExitRateRadps = 0.005f;
    static constexpr float kExitHold_s = 5.0f;

private:
    float timeUnderThreshold_s = 0.0f;
    PdController pd_;
    RwMapper rwMapper_;

    float lastTimestamp_s_ = 0.0f;
    bool hasLastTimestamp_ = false;
};
