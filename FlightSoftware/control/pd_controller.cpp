#include "pd_controller.h"

#include "math/vec3.h"

void PdController::setGains(const float kp[3], const float kd[3]) {
    math::vec3_copy(kp_, kp);
    math::vec3_copy(kd_, kd);
}

void PdController::compute(const float error[3], const float rate_error[3], float torque_Nm[3]) const {
    for (int i = 0; i < 3; i++) {
        torque_Nm[i] = kp_[i] * error[i] + kd_[i] * rate_error[i];
    }
}