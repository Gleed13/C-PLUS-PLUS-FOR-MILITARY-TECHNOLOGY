#pragma once

#include "DropPoint.hpp"

struct BallisticSolution {
    DropPoint dropPoint{};
    float fallTime = 0.0F;
    float horizontalDistance = 0.0F;
};
