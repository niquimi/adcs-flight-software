#include "flight_software.h"
#include "command_packets.h"

#include "math/attitude.h"
#include "math/mat3.h"
#include "math/vec3.h"

#include "tm/tm_snapshot.h"

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
    health_.reset();
    lastOrbitLog_s_ = -kOrbitLogInterval_s;
    bootHoldLogged_ = false;
    state_estimator_.reset();
    mode_forced_ = false;
    forced_mode_ = ModeId::Standby;
    last_tc_opcode_ = 0;
    last_tc_arg0_ = 0;
    health_event_count_ = 0;
}

void FlightSoftware::setBootStandbyDuration(float duration_s) {
    bootStandbyDuration_s_ = duration_s > 0.f ? duration_s : 0.f;
}

AttitudeCommand FlightSoftware::step(const SensorPacket& sensors) {
    const HealthReport health = health_.evaluateSensors(sensors);
    const ModeId current = fdir_.votedMode();
    FdirReport fdir = fdir_.evaluate(health);

    SpacecraftState state = state_estimator_.update(sensors, fdir.gate);
    health_.noteAttitude(state.attitude_valid);
    health_.noteCss(state.css_valid);
    fdir.force_safe = fdir.force_safe || health_.report().att_stale;

    if (bootStandbyDuration_s_ > 0.f && !bootHoldLogged_
        && state.timestamp_s < bootStandbyDuration_s_) {
        std::cout << "Boot Standby (launcher separation) until t="
                  << bootStandbyDuration_s_ << " s\n";
        bootHoldLogged_ = true;
    }

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
    in.mode_forced = mode_forced_;
    in.forced_mode = forced_mode_;

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
    const std::uint8_t hf = health_.report().flags();
    if (hf != 0 && health_event_count_ < 255) {
        ++health_event_count_;
    }
    fillTm(cmd, state, fdir, mode_forced_, health_.report(), last_tc_opcode_, last_tc_arg0_, health_event_count_);
    return cmd;
}

bool FlightSoftware::referenceValid(const SpacecraftState& state) const {
    switch (pointing_.target()) {
        case PointingTarget::Sun:
            if (active_->id() == ModeId::Pointing) {
                return !health_.report().css_stale;
            }
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

void FlightSoftware::applyTelecommand(std::uint8_t opcode, std::uint8_t arg0, std::uint8_t arg1) {
    last_tc_opcode_ = opcode;
    last_tc_arg0_ = arg0;

    switch (opcode) {
        case TC_FORCE_MODE:
            if (arg0 > static_cast<std::uint8_t>(ModeId::Safe)) {
                break;
            }
            mode_forced_ = true;
            forced_mode_ = static_cast<ModeId>(arg0);
            break;

        case TC_CLEAR_FORCE:
            mode_forced_ = false;
            break;

        case TC_CLEAR_FAULTS:
            fdir_.reset(active_->id());
            health_.reset();
            break;

        case TC_SET_POINTING_TARGET:
            if (arg0 == 0) {
                pointing_.setTarget(PointingTarget::Sun);
            } else if (arg0 == 1) {
                pointing_.setTarget(PointingTarget::Nadir);
            }
            break;

        case TC_RESET_ESTIMATOR:
            state_estimator_.reset();
            break;

        case TC_SET_GAINS:
        case TC_SET_THRESHOLDS:
            break;

        case TC_RESET_FSW:
            reset();
            break;

        default:
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
