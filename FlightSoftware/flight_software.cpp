#include "flight_software.h"

#include "math/attitude.h"
#include "math/mat3.h"
#include "math/vec3.h"

#include <iostream>

#ifdef ADCS_ENABLE_FAULT_INJECTION
ModeId& FlightSoftware::silModeReplica(int i) {
    return fdir_.modeReplica(i);
}
#endif

FlightSoftware::FlightSoftware() {
    reset();
}

void FlightSoftware::reset() {
    active_ = &standby_;
    active_->enter();
    fdir_.reset(ModeId::Standby);
    lastOrbitLog_s_ = -kOrbitLogInterval_s;
    bootHoldLogged_ = false;
    state_estimator_.reset();
}

void FlightSoftware::setBootStandbyDuration(float duration_s) {
    bootStandbyDuration_s_ = duration_s > 0.f ? duration_s : 0.f;
}

AttitudeCommand FlightSoftware::step(const SensorPacket& sensors) {
    SpacecraftState state = state_estimator_.update(sensors);

    if (state.timestamp_s - lastOrbitLog_s_ >= kOrbitLogInterval_s) {
        printEstCompare(state, sensors);
        lastOrbitLog_s_ = state.timestamp_s;
    }

    if (bootStandbyDuration_s_ > 0.f && !bootHoldLogged_
        && state.timestamp_s < bootStandbyDuration_s_) {
        std::cout << "Boot Standby (launcher separation) until t="
                  << bootStandbyDuration_s_ << " s\n";
        bootHoldLogged_ = true;
    }

    // Vote persistant ModeId
    const ModeId current = fdir_.votedMode();
    if (current != active_->id()) {
        active_->exit();
        modeFor(current);
        active_->enter();
        printModeChange(current, state.timestamp_s);
    }

    const EpsReport eps = eps_.evaluate(state.batteryLevel);

    fdir_.captureFlags(
        eps.request_safe,
        eps.allow_exit_safe,
        referenceValid(state)
    );
    bool force_safe = false;
    bool allow_exit_safe = false;
    bool ref_ok = false;
    fdir_.readFlags(force_safe, allow_exit_safe, ref_ok);

    const FdirReport fdir = fdir_.evaluate(state, current);
    force_safe = force_safe || fdir.force_safe;
    allow_exit_safe = allow_exit_safe && !fdir.force_safe;

    ModeDirector::Input in;
    in.current = current;
    in.timestamp_s = state.timestamp_s;
    in.boot_standby_duration_s = bootStandbyDuration_s_;
    in.rate_radps = math::vec3_norm(state.omega_radps);
    in.ref_ok = ref_ok;
    in.rates_settled = detumble_.ratesSettled();
    in.force_safe = force_safe;
    in.allow_exit_safe = allow_exit_safe;

    const ModeId next = director_.selectNextMode(in);
    if (next != active_->id()) {
        active_->exit();
        modeFor(next);
        active_->enter();
        printModeChange(next, state.timestamp_s);
    }

    fdir_.commitMode(active_->id());

    AttitudeCommand cmd = active_->update(state);
    cmd.active_mode = active_->id();
    return cmd;
}

bool FlightSoftware::referenceValid(const SpacecraftState& state) const {
    switch (pointing_.target()) {
        case PointingTarget::Sun:
            return state.css_valid;
        case PointingTarget::Nadir:
            return state.nadir_valid;
    }
    return false;
}

void FlightSoftware::modeFor(ModeId id) {
    switch (id) {
        case ModeId::Detumble:
            active_ = &detumble_;
            break;
        case ModeId::Standby:
            active_ = &standby_;
            break;
        case ModeId::Safe:
            active_ = &safe_;
            break;
        case ModeId::Pointing:
            active_ = &pointing_;
            break;
    }
}

void FlightSoftware::printModeChange(ModeId id, float timestamp_s) const {
    std::cout << "Mode=";

    switch (id) {
        case ModeId::Detumble:
            std::cout << "Detumble";
            break;
        case ModeId::Standby:
            std::cout << "Standby";
            break;
        case ModeId::Pointing:
            std::cout << "Pointing";
            break;
        case ModeId::Safe:
            std::cout << "Safe";
            break;
    }

    std::cout << " t=" << timestamp_s << " s\n";
}

void FlightSoftware::printEstCompare(const SpacecraftState& state, const SensorPacket& sensors) const {
    const float r_true[3] = {sensors.r_BN_x, sensors.r_BN_y, sensors.r_BN_z};
    float dr_vec[3] = {
        state.r_BN_N[0] - r_true[0],
        state.r_BN_N[1] - r_true[1],
        state.r_BN_N[2] - r_true[2],
    };
    const float dr = math::vec3_norm(dr_vec);
    const float sun_n_true[3] = {sensors.sun_N_x, sensors.sun_N_y, sensors.sun_N_z};
    const float mag_n_true[3] = {sensors.B_N_x, sensors.B_N_y, sensors.B_N_z};
    const float sun_n_deg = math::vec3_angle_deg(state.sun_N, sun_n_true);
    const float mag_n_deg = math::vec3_angle_deg(state.B_N, mag_n_true);

    float C_true[9];
    const float sigma_BN[3] = {sensors.sigma_BN_x, sensors.sigma_BN_y, sensors.sigma_BN_z};
    math::dcm_from_mrp(sigma_BN, C_true);
    float s_B_true[3];
    float b_B_true[3];
    math::mat3_mul_vec(C_true, sun_n_true, s_B_true);
    math::mat3_mul_vec(C_true, mag_n_true, b_B_true);
    const float sun_b_deg = math::vec3_angle_deg(state.css, s_B_true);
    const float mag_b_deg = math::vec3_angle_deg(state.mag, b_B_true);

    const float nr = math::vec3_norm(r_true);
    float nadir_n_deg = -1.f;
    float nadir_b_deg = -1.f;
    float triad_deg = -1.f;
    float att_deg = -1.f;
    if (nr > 1000.f) {
        float n_N_true[3];
        math::vec3_scale(r_true, -1.f / nr, n_N_true);
        nadir_n_deg = math::vec3_angle_deg(state.nadir_N, n_N_true);
        float n_B_true[3];
        math::mat3_mul_vec(C_true, n_N_true, n_B_true);
        if (state.nadir_valid) {
            nadir_b_deg = math::vec3_angle_deg(state.nadir_B, n_B_true);
        }
    }
    if (state.triad_valid) {
        triad_deg = math::dcm_geodesic_angle_deg(state.C_triad, C_true);
    }
    if (state.attitude_valid) {
        att_deg = math::dcm_geodesic_angle_deg(state.C_BN, C_true);
    }

    std::cout << "Est t=" << state.timestamp_s
              << " s |dr|=" << dr << " m"
              << " SunN=" << sun_n_deg << " deg"
              << " MagN=" << mag_n_deg << " deg"
              << " SunB=" << sun_b_deg << " deg"
              << " MagB=" << mag_b_deg << " deg";
    if (triad_deg < 0.f) {
        std::cout << " TRIAD=invalid";
    } else {
        std::cout << " TRIAD=" << triad_deg << " deg";
    }
    if (att_deg < 0.f) {
        std::cout << " Att=invalid";
    } else {
        std::cout << " Att=" << att_deg << " deg";
    }
    const float b_degph =
        math::vec3_norm(state.gyro_bias) * math::kRad2Deg * 3600.f;
    std::cout << " |b|=" << b_degph << " deg/h";
    std::cout << " NadirN=" << nadir_n_deg << " deg";
    if (nadir_b_deg < 0.f) {
        std::cout << " NadirB=invalid\n";
    } else {
        std::cout << " NadirB=" << nadir_b_deg << " deg\n";
    }
}
