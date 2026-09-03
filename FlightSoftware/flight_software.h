#pragma once

#include "types.h"
#include "sensor_packet.h"
#include "operation_mode.h"
#include "eps/eps_manager.h"
#include "fdir/fdir_manager.h"
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

    /** Operator telecommand. SIL-only inject is handled outside this class. */
    void applyTelecommand(std::uint8_t opcode, std::uint8_t arg0, std::uint8_t arg1);

#ifdef ADCS_ENABLE_FAULT_INJECTION
    ModeId& silModeReplica(int i);
#endif

private:
    StateEstimator state_estimator_;
    ModeDirector director_;
    EpsManager eps_;
    FdirManager fdir_;
    HealthMonitor health_;

    DetumbleMode detumble_;
    StandbyMode standby_;
    SafeMode safe_;
    PointingMode pointing_;

    OperationMode* active_ = nullptr;

    bool referenceValid(const SpacecraftState& state) const;
    void modeFor(ModeId id);
    void printModeChange(ModeId id, float timestamp_s) const;

    static constexpr float kOrbitLogInterval_s = 5.0f;

    float lastOrbitLog_s_ = -kOrbitLogInterval_s;
    float bootStandbyDuration_s_ = 0.f;
    bool bootHoldLogged_ = false;
    bool mode_forced_ = false;
    ModeId forced_mode_ = ModeId::Standby;
    std::uint8_t last_tc_opcode_ = 0;
    std::uint8_t last_tc_arg0_ = 0;
    std::uint8_t health_event_count_ = 0;
};
