#include "mode_director.h"

#include "modes/standby_mode.h"
#include "modes/detumble_mode.h"
#include "modes/safe_mode.h"

ModeId ModeDirector::selectNextMode(const Input& in) const {
    if (in.timestamp_s < in.boot_standby_duration_s) {
        return ModeId::Standby;
    }

    if (in.battery_level < kEnterSafeBattery) {
        return ModeId::Safe;
    }

    const float rate = in.rate_radps;
    const bool ref_ok = in.ref_ok;

    switch (in.current) {
        case ModeId::Standby:
            if (rate > StandbyMode::kEnterDetumbleRateRadps) {
                return ModeId::Detumble;
            }
            if (rate < DetumbleMode::kExitRateRadps && ref_ok) {
                return ModeId::Pointing;
            }
            return ModeId::Standby;

        case ModeId::Detumble:
            if (!in.rates_settled) {
                return ModeId::Detumble;
            }
            return ref_ok ? ModeId::Pointing : ModeId::Standby;

        case ModeId::Pointing:
            if (!ref_ok) {
                return ModeId::Standby;
            }
            return ModeId::Pointing;

        case ModeId::Safe:
            if (in.battery_level > SafeMode::kExitBatteryPercentage) {
                return (rate > StandbyMode::kEnterDetumbleRateRadps)
                    ? ModeId::Detumble
                    : ModeId::Standby;
            }
            return ModeId::Safe;
    }

    return in.current;
}