#pragma once

#include <cstddef>

#include "Coord.hpp"

struct TargetSelection {
    std::size_t targetIndex = 0;
    Coord predictedPosition{};
    Coord targetVelocity{};
    float estimatedArrivalTime = 0.0F;
};
