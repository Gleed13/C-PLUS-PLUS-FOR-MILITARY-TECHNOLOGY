#pragma once

#include "Coord.hpp"

enum class DroneStatus { Stopped = 0, Accelerating = 1, Decelerating = 2, Turning = 3, Moving = 4 };

struct DroneState {
    Coord position{};
    float direction = 0.0F;
    float speed = 0.0F;
    DroneStatus status = DroneStatus::Stopped;
};
