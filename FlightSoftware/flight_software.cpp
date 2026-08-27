#include "flight_software.h"

#include "math/attitude.h"
#include "math/mat3.h"
#include "math/vec3.h"

#include <iostream>

#ifdef ADCS_ENABLE_FAULT_INJECTION
ModeId& FlightSoftware::silModeReplica(int i) {
    return fdir_.modeReplica(i);
}
#endif

FlightSoftware::FlightSoftware() {
    reset();
}

void FlightSoftware::reset() {
    active_ = &standby_;
    active_->enter();
    fdir_.reset(ModeId::Standby);
    lastOrbitLog_s_ = -kOrbitLogInterval_s;
    bootHoldLogged_ = false;
    state_estimator_.reset();
}

void FlightSoftware::setBootStandbyDuration(float duration_s) {
    bootStandbyDuration_s_ = duration_s > 0.f ? duration_s : 0.f;
}

AttitudeCommand FlightSoftware::step(const SensorPacket& sensors) {
    SpacecraftState state = state_estimator_.update(sensors);

    if (bootStandbyDuration_s_ > 0.f && !bootHoldLogged_
        && state.timestamp_s < bootStandbyDuration_s_) {
        std::cout << "Boot Standby (launcher separation) until t="
                  << bootStandbyDuration_s_ << " s\n";
        bootHoldLogged_ = true;
    }

    // Vote persistant ModeId
    const ModeId current = fdir_.votedMode();
    if (current != active_->id()) {
        active_->exit();
        modeFor(current);
        active_->enter();
        printModeChange(current, state.timestamp_s);
    }

    const EpsReport eps = eps_.evaluate(state.batteryLevel);

    fdir_.captureFlags(
        eps.request_safe,
        eps.allow_exit_safe,
        referenceValid(state)
    );
    bool force_safe = false;
    bool allow_exit_safe = false;
    bool ref_ok = false;
    fdir_.readFlags(force_safe, allow_exit_safe, ref_ok);

    const FdirReport fdir = fdir_.evaluate(state, current);
    force_safe = force_safe || fdir.force_safe;
    allow_exit_safe = allow_exit_safe && !fdir.force_safe;

    ModeDirector::Input in;
    in.current = current;
    in.timestamp_s = state.timestamp_s;
    in.boot_standby_duration_s = bootStandbyDuration_s_;
    in.rate_radps = math::vec3_norm(state.omega_radps);
    in.ref_ok = ref_ok;
    in.rates_settled = detumble_.ratesSettled();
    in.force_safe = force_safe;
    in.allow_exit_safe = allow_exit_safe;

    const ModeId next = director_.selectNextMode(in);
    if (next != active_->id()) {
        active_->exit();
        modeFor(next);
        active_->enter();
        printModeChange(next, state.timestamp_s);
    }

    fdir_.commitMode(active_->id());

    AttitudeCommand cmd = active_->update(state);
    cmd.active_mode = active_->id();
    return cmd;
}

bool FlightSoftware::referenceValid(const SpacecraftState& state) const {
    switch (pointing_.target()) {
        case PointingTarget::Sun:
            return state.css_valid;
        case PointingTarget::Nadir:
            return state.nadir_valid;
    }
    return false;
}

void FlightSoftware::modeFor(ModeId id) {
    switch (id) {
        case ModeId::Detumble:
            active_ = &detumble_;
            break;
        case ModeId::Standby:
            active_ = &standby_;
            break;
        case ModeId::Safe:
            active_ = &safe_;
            break;
        case ModeId::Pointing:
            active_ = &pointing_;
            break;
    }
}

void FlightSoftware::printModeChange(ModeId id, float timestamp_s) const {
    std::cout << "Mode=";

    switch (id) {
        case ModeId::Detumble:
            std::cout << "Detumble";
            break;
        case ModeId::Standby:
            std::cout << "Standby";
            break;
        case ModeId::Pointing:
            std::cout << "Pointing";
            break;
        case ModeId::Safe:
            std::cout << "Safe";
            break;
    }

    std::cout << " t=" << timestamp_s << " s\n";
}
