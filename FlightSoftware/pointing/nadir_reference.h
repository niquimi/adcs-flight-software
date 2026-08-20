#pragma once

#include "reference_generator.h"

class NadirReference : public ReferenceGenerator {
public:
    PointingReference compute(const SpacecraftState& state) override;
};