#pragma once

#include "Coord.hpp"
#include "DroneState.hpp"

struct SimulationStep
{
    DroneState drone{};
    int targetIndex = -1;
    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
};
