#pragma once

#include <optional>

#include "models/Ammo.hpp"
#include "models/BallisticSolution.hpp"
#include "models/Coord.hpp"
#include "models/DroneConfig.hpp"

class IBallisticSolver {
public:
    virtual std::optional<BallisticSolution> solve(const DroneConfig& drone_config, const Coord& target_position, const Ammo& ammo) = 0;
    virtual ~IBallisticSolver() = default;
};
