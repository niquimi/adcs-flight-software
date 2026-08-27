#pragma once

#include <cstdint>

enum class ModeId : std::uint8_t {
    Standby = 0,
    Detumble = 1,
    Pointing = 2,
    Safe = 3,
};

/** Estimated spacecraft state used by modes/controllers. */
struct SpacecraftState {
    float timestamp_s = 0.f;
    float omega_radps[3] = {0.f, 0.f, 0.f};
    float css[3] = {0.f, 0.f, 0.f};  // unit sun_B from CssWls
    float mag[3] = {0.f, 0.f, 0.f};
    float batteryLevel = 0.0f;
    float r_BN_N[3] = {0.f, 0.f, 0.f};
    float sun_N[3] = {0.f, 0.f, 0.f};
    bool css_valid = false;
    float B_N[3] = {0.f, 0.f, 0.f};
    float C_BN[9] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float C_triad[9] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float gyro_bias[3] = {0.f, 0.f, 0.f};  // rad/s, body, AttitudeFilter
    bool attitude_valid = false;
    bool triad_valid = false;
    float nadir_N[3] = {0.f, 0.f, 0.f};  // unit, toward Earth, from Kepler
    float nadir_B[3] = {0.f, 0.f, 0.f};  // C_BN * nadir_N
    bool nadir_valid = false;

};

/** Actuator command returned by FlightSoftware::step to the SIL layer. */
struct AttitudeCommand {
    bool apply_rw = false;
    bool apply_mtb = false;
    float rw_torque_Nm[3] = {0.f, 0.f, 0.f};
    float mtb_dipole_Am2[3] = {0.f, 0.f, 0.f};
    ModeId active_mode = ModeId::Standby;
    std::uint8_t validity_flags = 0;
    std::uint16_t tmr_mismatch_count = 0;
    float gyro_bias_degph = 0.f;
    float sigma_BN_est[3] = {0.f, 0.f, 0.f};
    bool attitude_valid = false;
};
