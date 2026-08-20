#pragma once

#include "reference_generator.h"

class SunReference : public ReferenceGenerator {
public:
    PointingReference compute(const SpacecraftState& state) override;
};