#include "fault_injector.h"

#include <cstdlib>
#include <iostream>

static float envSeconds(const char* name) {
    const char* s = std::getenv(name);
    if (s == nullptr || s[0] == '\0') {
        return -1.f;
    }
    return std::strtof(s, nullptr);
}

FaultInjector::FaultInjector()
    : mode_at_s_(envSeconds("ADCS_INJECT_MODE_AT"))
    , soc_at_s_(envSeconds("ADCS_INJECT_SOC_AT"))
{}

void FaultInjector::apply(SensorPacket& sensors, FlightSoftware& fsw) {
    if (!mode_done_ && mode_at_s_ >= 0.f && sensors.timestamp_s >= mode_at_s_) {
        auto& r = fsw.silModeReplica(0);
        const auto before = static_cast<std::uint8_t>(r);
        r = static_cast<ModeId>(before ^ 0x04);
        std::cout << "INJECT ModeId replica0 " << static_cast<int>(before)
                  << "->" << static_cast<int>(before ^ 0x04)
                  << " t=" << sensors.timestamp_s << "s\n";
        mode_done_ = true;
    }

    if (!soc_done_ && soc_at_s_ >= 0.f && sensors.timestamp_s >= soc_at_s_) {
        sensors.batteryLevel = 0.10f;
        std::cout << "INJECT SOC=0.10 t=" << sensors.timestamp_s << "s\n";
        soc_done_ = true;
    }
}