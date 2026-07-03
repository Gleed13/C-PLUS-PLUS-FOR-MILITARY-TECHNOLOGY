#pragma once

#include <optional>

#include "interfaces/IBallisticSolver.hpp"

class AnalyticalBallisticSolver : public IBallisticSolver {
public:
    const float kGravity = 9.81f;

    std::optional<BallisticSolution> solve(const DroneConfig& drone_config, const Coord& target_position, const Ammo& ammo) override;
    std::optional<float> getAmmoHorizontalDistance(const DroneConfig& drone_config, const Ammo& ammo) override;

private:
    bool tryCalculateFreeFallTime(float zd, float attackSpeed, float mass, float drag, float lift, float* t);
    bool tryCalculateHorizontalDistance(float t, float attackSpeed, float mass, float drag, float lift, float* h);
    bool tryCalculateDropPoint(float xd, float yd, float xt, float yt, float accelerationPath, float h, DropPoint* dropPoint);
};
