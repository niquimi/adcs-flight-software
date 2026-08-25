#include "eps_manager.h"

EpsReport EpsManager::evaluate(float soc) const {
    EpsReport r;
    r.soc = soc;
    r.request_safe = soc < kEnterSafeSoc;
    r.allow_exit_safe = soc > kExitSafeSoc;
    return r;
}

float EpsManager::modeLoadW(ModeId mode) {
    // Matches BasiliskSim/bridge/battery_soc_model.py P_LOAD_W. Not applied onboard.
    switch (mode) {
        case ModeId::Standby:
            return 28.f;
        case ModeId::Detumble:
            return 55.f;
        case ModeId::Pointing:
            return 50.f;
        case ModeId::Safe:
            return 24.f;
    }
    return 28.f;
}