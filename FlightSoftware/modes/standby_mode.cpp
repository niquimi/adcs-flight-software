#include "standby_mode.h"

ModeId StandbyMode::id() const {
    return ModeId::Standby;
}

void StandbyMode::enter() {

}

void StandbyMode::exit() {

}

AttitudeCommand StandbyMode::update(const SpacecraftState&) {
    AttitudeCommand cmd;
    // Zero residual RW / MTB commands from the previous mode.
    cmd.apply_mtb = true;
    cmd.apply_rw = true;
    cmd.active_mode = ModeId::Standby;
    return cmd;
}