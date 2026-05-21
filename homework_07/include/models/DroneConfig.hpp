#pragma once

#include "Coord.hpp"

struct DroneConfig
{
    Coord startPos;
    float altitude;
    float attackSpeed;
    float accelPath;
    char ammoName[32];
};