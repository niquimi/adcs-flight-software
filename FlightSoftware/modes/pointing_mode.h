#pragma once

#include "operation_mode.h"
#include "types.h"
#include "control/pd_controller.h"
#include "actuators/rw_mapper.h"
#include "sun_reference.h"
#include "nadir_reference.h"

enum class PointingTarget : std::uint8_t {
    Sun = 0,
    Nadir = 1,
};

class PointingMode : public OperationMode {
public:
    PointingMode();

    ModeId id() const override;

    void enter() override;
    void exit() override;

    AttitudeCommand update(const SpacecraftState& state) override;

    void setTarget(PointingTarget target);
    PointingTarget target() const;

private:
    PointingTarget target_ = PointingTarget::Nadir;
    SunReference sunReference_;
    NadirReference nadirReference_;
    PdController pd_;
    RwMapper rwMapper_;
};
