#pragma once

#include "sensor_packet.h"
#include "flight_software.h"

class FaultInjector {
    public:
        FaultInjector();  // reads getenv
    
        void apply(SensorPacket& sensors, FlightSoftware& fsw);

        /** One-shot from a telecommand. arg0: 1 = ModeId replica XOR, 2 = SOC 0.10. */
        void fire(std::uint8_t arg0, SensorPacket& sensors, FlightSoftware& fsw);
    
    private:
        float mode_at_s_ = -1.f;
        float soc_at_s_  = -1.f;
        bool mode_done_ = false;
        bool soc_done_  = false;
    };