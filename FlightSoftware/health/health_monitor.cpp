#include "health_monitor.h"

void HealthMonitor::reset() {
    last_t_s_ = 0.f;
    has_t_ = false;
    att_invalid_streak_ = 0;
    css_invalid_streak_ = 0;
    report_ = HealthReport{};
}

const HealthReport& HealthMonitor::report() const {
    return report_;
}

void HealthMonitor::noteAttitude(bool attitude_valid) {
    if (attitude_valid) {
        att_invalid_streak_ = 0;
    } else if (att_invalid_streak_ < kAttInvalidN) {
        ++att_invalid_streak_;
    }
    report_.att_stale = (att_invalid_streak_ >= kAttInvalidN);
}

void HealthMonitor::noteCss(bool css_valid) {
    if (css_valid) {
        css_invalid_streak_ = 0;
    } else if (css_invalid_streak_ < kCssInvalidN) {
        ++css_invalid_streak_;
    }
    report_.css_stale = (css_invalid_streak_ >= kCssInvalidN);
}

HealthReport HealthMonitor::evaluateSensors(const SensorPacket& sensors) {
    HealthReport hr;

    if (has_t_) {
        float dt = sensors.timestamp_s - last_t_s_;
        if (dt < 0) hr.dt_back = true;
        if (dt > kDtMax_s) hr.dt_skip = true;
    }
    last_t_s_ = sensors.timestamp_s;
    has_t_ = true;

    if (sensors.gyro_x > kGyroMax_radps || sensors.gyro_x < -kGyroMax_radps ||
        sensors.gyro_y > kGyroMax_radps || sensors.gyro_y < -kGyroMax_radps ||
        sensors.gyro_z > kGyroMax_radps || sensors.gyro_z < -kGyroMax_radps) {
            hr.gyro_oor = true;
    }

    const float b2 = 
        sensors.mag_x * sensors.mag_x +
        sensors.mag_y * sensors.mag_y +
        sensors.mag_z * sensors.mag_z;
    if (b2 < kMagMin_nT * kMagMin_nT || b2 > kMagMax_nT * kMagMax_nT) {
        hr.mag_oor = true;
    }

    if (sensors.css_px < kCssMin || sensors.css_px > kCssMax ||
        sensors.css_mx < kCssMin || sensors.css_mx > kCssMax ||
        sensors.css_py < kCssMin || sensors.css_py > kCssMax ||
        sensors.css_my < kCssMin || sensors.css_my > kCssMax ||
        sensors.css_pz < kCssMin || sensors.css_pz > kCssMax ||
        sensors.css_mz < kCssMin || sensors.css_mz > kCssMax) {
        hr.css_range = true;
    }

    if ((sensors.css_mx > kCssOppMin && sensors.css_px > kCssOppMin) ||
        (sensors.css_my > kCssOppMin && sensors.css_py > kCssOppMin) ||
        (sensors.css_mz > kCssOppMin && sensors.css_pz > kCssOppMin)) {
        hr.css_incoh = true;
    }


    report_ = hr;
    return hr;
}