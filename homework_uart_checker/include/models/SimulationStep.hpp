#pragma once

#include "Coord.hpp"
#include "models/DroneTelemetry.hpp"

struct SimulationStep {
    DroneTelemetry droneTelemetry{};
    int targetIndex = -1;
    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
    float timeSecSinceStart = 0.0F;
};
