#pragma once

#include <cstdint>
#include <sensor_packet.h>

// Error            invalidate      force_safe     healt_flags
// dt_back          gyro+mag+css    yes            0x01
// dt_skip          gyro            no             0x02
// gyro_oor         gyro            yes            0x04
// mag_oor          mag             no             0x08
// css_range        css             no             0x10
// css_incoh        css             no             0x20
// att_stale        -               yes            0x40
// css_stale        -               no             0x80

struct HealthReport {
    bool dt_back = false;
    bool dt_skip = false;
    bool gyro_oor = false;
    bool mag_oor = false;
    bool css_range = false;
    bool css_incoh = false;
    bool att_stale = false;
    bool css_stale = false;
    
    std::uint8_t flags() const {
        std::uint8_t f = 0;
        if (dt_back)    f |= 0x01;
        if (dt_skip)    f |= 0x02;
        if (gyro_oor)   f |= 0x04;
        if (mag_oor)    f |= 0x08;
        if (css_range)  f |= 0x10;
        if (css_incoh)  f |= 0x20;
        if (att_stale)  f |= 0x40;
        if (css_stale)  f |= 0x80;
        return f;
    }
};

class HealthMonitor {
public:
    static constexpr float kDtMax_s = 1.0f;
    static constexpr float kGyroMax_radps = 2.0f;
    static constexpr float kMagMin_nT = 5.0e3f;
    static constexpr float kMagMax_nT = 1.0e5f;
    static constexpr float kCssMin = -0.06f;
    static constexpr float kCssMax = 2.5f;
    static constexpr float kCssOppMin = 0.2f;
    static constexpr int   kAttInvalidN = 50;
    static constexpr int   kCssInvalidN = 50;

    void reset();
    HealthReport evaluateSensors(const SensorPacket& sensors);
    void noteAttitude(bool attitude_valid);
    void noteCss(bool css_valid);
    const HealthReport& report() const;

private:
    float last_t_s_ = 0.f;
    bool has_t_ = false;
    int att_invalid_streak_ = 0;
    int css_invalid_streak_ = 0;
    HealthReport report_{};
};