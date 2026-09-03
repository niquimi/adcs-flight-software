#pragma once

#include "types.h"
#include "fdir/tmr.h"
#include "health/health_monitor.h"

struct SensorGate {
    bool use_gyro = true;
    bool use_mag = true;
    bool use_css = true;
};

struct FdirReport {
    bool force_safe = false;
    bool tmr_mismatch = false;
    bool tmr_no_majority = false;
    SensorGate gate = {};
    std::uint32_t tmr_mismatch_count = 0;
};

class FdirManager {
public:
    void reset(ModeId boot = ModeId::Standby);

    ModeId votedMode();
    void commitMode(ModeId id);

    void captureFlags(bool force_safe, bool allow_exit_safe, bool ref_ok);
    void readFlags(bool& force_safe, bool& allow_exit_safe, bool& ref_ok);

    FdirReport evaluate(const HealthReport&) const;

    ModeId& modeReplica(int i) { return mode_.replica(i); }

private:
    static bool modeIdValid(ModeId id);

    Tmr<ModeId> mode_;
    Tmr<bool> force_safe_;
    Tmr<bool> allow_exit_safe_;
    Tmr<bool> ref_ok_;

    bool tmr_mismatch_ = false;
    bool tmr_no_majority_ = false;
    std::uint32_t tmr_mismatch_count_ = 0;
};
