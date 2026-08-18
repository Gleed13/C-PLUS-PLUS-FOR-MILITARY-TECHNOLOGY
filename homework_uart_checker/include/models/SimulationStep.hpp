#pragma once

#include <optional>
#include "Coord.hpp"
#include "models/DroneTelemetry.hpp"

struct SimulationStep {
    DroneTelemetry droneTelemetry{};
    int targetIndex = -1;
    std::optional<Coord> dropPoint{};
    Coord aimPoint{};
    std::optional<Coord> predictedTarget{};
    float timeSecSinceStart = 0.0F;
};
