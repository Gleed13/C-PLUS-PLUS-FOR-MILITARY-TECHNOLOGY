#pragma once

#include "models/Coord.hpp"
#include "models/DroneState.hpp"

namespace ControlUtils {
    float angleToTarget(const Coord& target_position, const DroneState& drone_);
}