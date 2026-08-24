#pragma once

#include "types.h"
#include "sensor_packet.h"
#include "operation_mode.h"
#include "estimation/state_estimator.h"
#include "modes/detumble_mode.h"
#include "modes/standby_mode.h"
#include "modes/safe_mode.h"
#include "modes/pointing_mode.h"
#include "mode_director.h"

/**
 * Flight Software orchestrator — single entry point from the SIL loop.
 *
 * Contract: sil_connection calls step() once per SensorPacket.
 */
class FlightSoftware {
public:
    FlightSoftware();

    void reset();

    AttitudeCommand step(const SensorPacket& sensors);

    /** Hold Standby until timestamp_s reaches this (launcher separation). 0 = off. */
    void setBootStandbyDuration(float duration_s);


private:
    StateEstimator state_estimator_;
    ModeDirector director_;

    DetumbleMode detumble_;
    StandbyMode standby_;
    SafeMode safe_;
    PointingMode pointing_;

    OperationMode* active_ = nullptr;

    bool referenceValid(const SpacecraftState& state) const;
    void modeFor(ModeId id);
    void printModeChange(ModeId id, float timestamp_s) const;
    void printEstCompare(const SpacecraftState& state, const SensorPacket& sensors) const;

    static constexpr float kOrbitLogInterval_s = 5.0f;

    float lastOrbitLog_s_ = -kOrbitLogInterval_s;
    float bootStandbyDuration_s_ = 0.f;
    bool bootHoldLogged_ = false;
};
