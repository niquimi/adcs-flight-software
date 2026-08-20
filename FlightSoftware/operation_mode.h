#pragma once

#include "types.h"

/**
 * Common interface for all operation modes (Standby, Detumble, Pointing, Safe, ...).
 * Implementations live under modes/.
 */
class OperationMode {
public:
    virtual ~OperationMode() = default;

    virtual ModeId id() const = 0;

    virtual void enter() {}
    virtual void exit() {}

    /** Produce an actuator command from the current estimated state. */
    virtual AttitudeCommand update(const SpacecraftState& state) = 0;
};
