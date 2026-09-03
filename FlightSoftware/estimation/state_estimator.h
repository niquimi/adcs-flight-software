#pragma once

#include "types.h"
#include "sensor_packet.h"
#include "orbit_propagator.h"
#include "sun_model.h"
#include "dipole_mag_model.h"
#include "triad.h"
#include "css_wls.h"
#include "attitude_filter.h"
#include "fdir/fdir_manager.h"

/** Raw sensors + onboard models → SpacecraftState. */
class StateEstimator {
public:
    void reset();

    SpacecraftState update(const SensorPacket& sensors, const SensorGate& gate);

private:
    OrbitPropagator orbit_propagator_;
    SunModel sun_model_;
    DipoleMagModel dipole_mag_model_;
    CssWls css_wls_;
    Triad triad_;
    AttitudeFilter attitude_filter_;
};
