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
};
#pragma pack(pop)

static_assert(sizeof(SensorPacket) == 56, "SensorPacket must be exactly 56 bytes");
static_assert(sizeof(float) == 4, "float must be 4 bytes for Python struct '<27f' compatibility");

constexpr std::size_t kSensorPacketSize = sizeof(SensorPacket);
constexpr uint16_t kDefaultSilPort = 5557;
