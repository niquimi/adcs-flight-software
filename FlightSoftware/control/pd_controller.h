#pragma once

/** Generic PD controller (setpoint + gains -> torque). To be implemented. */
class PdController {
public:
    void setGains(const float kp[3], const float kd[3]);

    /** error[3], rate_error[3] -> torque_Nm[3] */
    void compute(
        const float error[3],
        const float rate_error[3],
        float torque_Nm[3]
    ) const;

private:
    float kp_[3] = {.0f, .0f, .0f};
    float kd_[3] = {.0f, .0f, .0f};
};
