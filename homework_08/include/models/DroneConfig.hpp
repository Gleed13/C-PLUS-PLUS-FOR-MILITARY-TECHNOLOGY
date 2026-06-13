#pragma once

#include <string>

#include "Coord.hpp"

struct DroneConfig
{
    Coord startPos;
    float altitude;
    float attackSpeed;
    float accelPath;
    std::string ammoName;
};