#pragma once

#include <cstddef>

#include "Coord.hpp"

struct TargetSelection
{
    std::size_t targetIndex = 0;
    Coord predictedPosition{};
    float estimatedArrivalTime = 0.0F;
};
