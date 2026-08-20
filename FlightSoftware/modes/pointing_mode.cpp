#include "pointing_mode.h"

#include "math/vec3.h"

namespace {
constexpr float kKp = 0.5f;
constexpr float kKd = 10.0f;
}  // namespace

PointingMode::PointingMode() {
    const float kp[3] = {kKp, kKp, 0.f};
    const float kd[3] = {kKd, kKd, kKd};
    pd_.setGains(kp, kd);
}

ModeId PointingMode::id() const {
    return ModeId::Pointing;
}

void PointingMode::enter() {
}

void PointingMode::exit() {
}

void PointingMode::setTarget(PointingTarget target) {
    target_ = target;
}

PointingTarget PointingMode::target() const {
    return target_;
}

AttitudeCommand PointingMode::update(const SpacecraftState& state) {
    PointingReference ref;

    switch (target_) {
        case PointingTarget::Sun:
            ref = sunReference_.compute(state);
            break;
        case PointingTarget::Nadir:
            ref = nadirReference_.compute(state);
            break;
    }

    const float error[3] = {
        ref.valid ? ref.attitude_error[0] : 0.f,
        ref.valid ? ref.attitude_error[1] : 0.f,
        ref.valid ? ref.attitude_error[2] : 0.f,
    };
    float rate_error[3];
    math::vec3_negate(state.omega_radps, rate_error);

    float torque_nm[3];
    pd_.compute(error, rate_error, torque_nm);
    AttitudeCommand cmd = rwMapper_.mapTorque(torque_nm);
    cmd.active_mode = ModeId::Pointing;
    return cmd;
}
