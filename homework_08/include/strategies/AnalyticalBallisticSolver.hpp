#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "models/Coord.hpp"
#include "models/DropPoint.hpp"

class AnalyticalBallisticSolver : public IBallisticSolver {
public:
    const float kGravity = 9.81f;

    bool trySolve(const DroneConfig* droneConfig, const Coord* targetPos, const Ammo* ammo, DropPoint* dropPoint) override;

private:
    bool tryCalculateFreeFallTime(float zd, float attackSpeed, float mass, float drag, float lift, float* t);
    bool tryCalculateHorizontalDistance(float t, float attackSpeed, float mass, float drag, float lift, float* h);
    bool tryCalculateDropPoint(float xd, float yd, float xt, float yt, float accelerationPath, float h, DropPoint* dropPoint);
};