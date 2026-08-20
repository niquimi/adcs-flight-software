#include "detumble_mode.h"

#include "math/vec3.h"

namespace {
    constexpr float kKp[3] = {.0f, .0f, .0f};
    constexpr float kKd[3] = {10.0f, 9.0f, 7.5f};
}

DetumbleMode::DetumbleMode() {
    pd_.setGains(kKp, kKd);
}

ModeId DetumbleMode::id() const {
    return ModeId::Detumble;
}

void DetumbleMode::enter() {
    timeUnderThreshold_s = 0.0f;
    hasLastTimestamp_ = false;
}

void DetumbleMode::exit() {

}

AttitudeCommand DetumbleMode::update(const SpacecraftState& state) {
    // Updates time dt
    float dt = 0.0f;
    
    if (hasLastTimestamp_) {
        dt = state.timestamp_s - lastTimestamp_s_;
    } else {
        hasLastTimestamp_ = true;
    }
    lastTimestamp_s_ = state.timestamp_s;

    // Compute error and map torque to command
    const float error[3] = {.0f, .0f, .0f};
    float rate_error[3];
    math::vec3_negate(state.omega_radps, rate_error);

    float torque_Nm[3];
    pd_.compute(error, rate_error, torque_Nm);

    AttitudeCommand cmd = rwMapper_.mapTorque(torque_Nm);
    cmd.active_mode = ModeId::Detumble;

    const float rateMagnitude = math::vec3_norm(state.omega_radps);
    if (rateMagnitude < kExitRateRadps) {
        timeUnderThreshold_s += dt;
    } else {
        timeUnderThreshold_s = 0.f;
    }

    return cmd;
}

bool DetumbleMode::ratesSettled() const {
    return timeUnderThreshold_s >= kExitHold_s;
}
