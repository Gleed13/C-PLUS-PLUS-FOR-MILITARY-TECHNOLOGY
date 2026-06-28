#pragma once

#include "models/Coord.hpp"
#include "models/DroneState.hpp"

struct DroneTelemetry {
    Coord position{};
    float direction = 0.0F;
    float speed = 0.0F;
    DroneStatus status = DroneStatus::Stopped;
    float timeSecSinceStart = 0.0F;
};