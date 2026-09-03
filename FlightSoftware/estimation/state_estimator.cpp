#include "state_estimator.h"

#include "math/mat3.h"
#include "math/vec3.h"

namespace {
constexpr float kMinRadius_m = 1000.0f;

void setNadir(SpacecraftState& state) {
    if (!math::vec3_normalize(state.r_BN_N, state.nadir_N, kMinRadius_m)) {
        return;
    }
    math::vec3_negate(state.nadir_N, state.nadir_N);
    if (!state.attitude_valid) {
        return;
    }
    math::mat3_mul_vec(state.C_BN, state.nadir_N, state.nadir_B);
    state.nadir_valid = true;
}
}  // namespace

void StateEstimator::reset() {
    orbit_propagator_.reset();
    sun_model_.reset();
    dipole_mag_model_.reset();
    css_wls_.reset();
    triad_.reset();
    attitude_filter_.reset();
}

SpacecraftState StateEstimator::update(const SensorPacket& sensors, const SensorGate& gate) {
    SpacecraftState state;
    state.timestamp_s = sensors.timestamp_s;
    state.batteryLevel = sensors.batteryLevel;

    if (gate.use_gyro) {
        state.omega_radps[0] = sensors.gyro_x;
        state.omega_radps[1] = sensors.gyro_y;
        state.omega_radps[2] = sensors.gyro_z;
    }

    if (gate.use_css) {
        const float css_raw[6] = {
            sensors.css_px,
            sensors.css_mx,
            sensors.css_py,
            sensors.css_my,
            sensors.css_pz,
            sensors.css_mz,
        };
        state.css_valid = css_wls_.update(css_raw, state.css);
    }

    if (gate.use_mag) {
        state.mag[0] = sensors.mag_x;
        state.mag[1] = sensors.mag_y;
        state.mag[2] = sensors.mag_z;
    }
    
    orbit_propagator_.update(sensors.timestamp_s, state.r_BN_N);
    sun_model_.update(sensors.timestamp_s, state.sun_N);
    dipole_mag_model_.update(sensors.timestamp_s, state.r_BN_N, state.B_N);
    float C_triad[9];
    const bool triad_ok = triad_.update(
        state.css,
        state.mag,
        state.sun_N,
        state.B_N,
        C_triad
    );
    state.triad_valid = triad_ok;
    if (triad_ok) {
        math::mat3_copy(state.C_triad, C_triad);
    }
    state.attitude_valid = attitude_filter_.update(
        sensors.timestamp_s,
        state.omega_radps,
        C_triad,
        triad_ok,
        state.C_BN
    );
    attitude_filter_.gyroBias(state.gyro_bias);

    if (gate.use_gyro) {
        state.omega_radps[0] -= state.gyro_bias[0];
        state.omega_radps[1] -= state.gyro_bias[1];
        state.omega_radps[2] -= state.gyro_bias[2];
    }
    setNadir(state);
    return state;
}
