#include "fdir_manager.h"

bool FdirManager::modeIdValid(ModeId id) {
    return static_cast<std::uint8_t>(id)
        <= static_cast<std::uint8_t>(ModeId::Safe);
}

void FdirManager::reset(ModeId boot) {
    mode_.set(boot);
    force_safe_.set(false);
    allow_exit_safe_.set(false);
    ref_ok_.set(false);
    tmr_mismatch_ = false;
    tmr_no_majority_ = false;
    tmr_mismatch_count_ = 0;
}

ModeId FdirManager::votedMode() {
    ModeId id = ModeId::Standby;
    tmr_mismatch_ = mode_.readAndRepair(id);

    tmr_no_majority_ = !mode_.hasMajority() || !modeIdValid(id);
    if (tmr_mismatch_) {
        ++tmr_mismatch_count_;
    }
    if (tmr_no_majority_) {
        id = ModeId::Safe;
        mode_.set(id);
    }
    return id;
}

void FdirManager::commitMode(ModeId id) {
    mode_.set(id);
}

void FdirManager::captureFlags(bool force_safe, bool allow_exit_safe, bool ref_ok) {
    force_safe_.set(force_safe);
    allow_exit_safe_.set(allow_exit_safe);
    ref_ok_.set(ref_ok);
}

void FdirManager::readFlags(bool& force_safe, bool& allow_exit_safe, bool& ref_ok) {
    if (force_safe_.readAndRepair(force_safe)) {
        ++tmr_mismatch_count_;
        tmr_mismatch_ = true;
    }
    if (allow_exit_safe_.readAndRepair(allow_exit_safe)) {
        ++tmr_mismatch_count_;
        tmr_mismatch_ = true;
    }
    if (ref_ok_.readAndRepair(ref_ok)) {
        ++tmr_mismatch_count_;
        tmr_mismatch_ = true;
    }
}

FdirReport FdirManager::evaluate(const SpacecraftState&, ModeId) const {
    FdirReport r;
    r.force_safe = tmr_no_majority_;
    r.tmr_mismatch = tmr_mismatch_;
    r.tmr_no_majority = tmr_no_majority_;
    r.tmr_mismatch_count = tmr_mismatch_count_;
    return r;
}