#pragma once

#include <cstddef>
#include <cstdint>

#pragma pack(push, 1)
struct SensorPacket {
    float timestamp_s;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float css_px;
    float css_mx;
    float css_py;
    float css_my;
    float css_pz;
    float css_mz;
    float batteryLevel;
    // Verification data, not a sensor. FSW estimators must not use these fields.
    float eclipse_shadow;
    float r_BN_x;
    float r_BN_y;
    float r_BN_z;
    // Inertial unit references from the sim (SIL). Zero → FSW uses onboard models.
    float sun_N_x;
    float sun_N_y;
    float sun_N_z;
    float B_N_x;
    float B_N_y;
    float B_N_z;
    // Verification: Basilisk MRP attitude. FSW TRIAD must not use these fields.
    float sigma_BN_x;
    float sigma_BN_y;
    float sigma_BN_z;
};
#pragma pack(pop)

static_assert(sizeof(SensorPacket) == 108, "SensorPacket must be exactly 108 bytes");
static_assert(sizeof(float) == 4, "float must be 4 bytes for Python struct '<27f' compatibility");

constexpr std::size_t kSensorPacketSize = sizeof(SensorPacket);
constexpr uint16_t kDefaultSilPort = 5557;
