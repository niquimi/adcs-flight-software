#pragma once

#include "types.h"
#include "fdir/fdir_manager.h"
#include "health/health_monitor.h"
#include "math/attitude.h"
#include "math/vec3.h"
#include "math/constants.h"

inline void fillTm(
    AttitudeCommand& cmd,
    const SpacecraftState& state,
    const FdirReport& fdir,
    bool mode_forced,
    const HealthReport& health,
    std::uint8_t last_tc_opcode,
    std::uint8_t last_tc_arg0,
    std::uint8_t health_event_count
) {
    cmd.gyro_bias_degph =
        math::vec3_norm(state.gyro_bias) * math::kRad2Deg * 3600.f;

    cmd.validity_flags = 0;
    if (state.css_valid) cmd.validity_flags |= 1;
    if (state.nadir_valid) cmd.validity_flags |= 2;
    if (state.attitude_valid) cmd.validity_flags |= 4;
    if (state.triad_valid) cmd.validity_flags |= 8;
    if (fdir.tmr_mismatch) cmd.validity_flags |= 16;
    if (fdir.tmr_no_majority) cmd.validity_flags |= 32;
    if (mode_forced) cmd.validity_flags |= 64;

    cmd.tmr_mismatch_count = static_cast<std::uint16_t>(
        fdir.tmr_mismatch_count > 65535u ? 65535u : fdir.tmr_mismatch_count);

    cmd.attitude_valid = state.attitude_valid;
    cmd.health_flags = health.flags();
    cmd.last_tc_opcode = last_tc_opcode;
    cmd.last_tc_arg0 = last_tc_arg0;
    cmd.health_event_count = health_event_count;

    if (state.attitude_valid) {
        math::mrp_from_dcm(state.C_BN, cmd.sigma_BN_est);
    }
}