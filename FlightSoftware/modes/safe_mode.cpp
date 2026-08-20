#include "safe_mode.h"

#include "math/vec3.h"


namespace {
    constexpr float kKp[3] = {0.5f, 0.5f, 0.0f};
    constexpr float kKd[3] = {10.0f, 9.0f, 7.5f};
}

SafeMode::SafeMode() {
    pd_.setGains(kKp, kKd);
}

ModeId SafeMode::id() const {
    return ModeId::Safe;
}

void SafeMode::enter() {
}

void SafeMode::exit() {

}

AttitudeCommand SafeMode::update(const SpacecraftState &state) {
    const PointingReference ref = sunReference_.compute(state);
    const float error[3] = {
        ref.valid ? ref.attitude_error[0] : 0.f,
        ref.valid ? ref.attitude_error[1] : 0.f,
        ref.valid ? ref.attitude_error[2] : 0.f
    };    

    float rate_error[3];
    math::vec3_negate(state.omega_radps, rate_error);

    float torque_Nm[3];
    pd_.compute(error, rate_error, torque_Nm);

    AttitudeCommand cmd = rwMapper_.mapTorque(torque_Nm);
    cmd.active_mode = ModeId::Safe;
    return cmd;
}