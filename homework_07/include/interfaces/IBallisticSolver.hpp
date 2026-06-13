#pragma once

#include "models/Coord.hpp"
#include "models/Ammo.hpp"
#include "models/DroneConfig.hpp"
#include "models/DropPoint.hpp"

class IBallisticSolver {
public:
    virtual bool trySolve(const DroneConfig* droneConfig, const Coord* targetPos, const Ammo* ammo, DropPoint* dropPoint) = 0;
    virtual ~IBallisticSolver() = default;
};