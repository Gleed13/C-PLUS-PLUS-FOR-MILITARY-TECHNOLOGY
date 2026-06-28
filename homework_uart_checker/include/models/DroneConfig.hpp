#pragma once

#include <string>

#include "Coord.hpp"

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    std::string ammoName;
    float arrayTimeStep;
    float simTimeStep;
    float physicsTimeStep;
    float hitRadius;
    float timeScale;
    float angularSpeed;
    float turnThreshold;
};