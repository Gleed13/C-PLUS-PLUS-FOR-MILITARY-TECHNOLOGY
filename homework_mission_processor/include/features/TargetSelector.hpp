#pragma once

#include <optional>

#include "features/TrajectoryEstimator.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/Ammo.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/TargetSelection.hpp"

class TargetSelector final
{
public:
    std::optional<TargetSelection> select(
        const DroneState& drone,
        float simulation_time,
        const DroneConfig& config,
        const Ammo& ammo,
        const ITargetProvider& target_provider,
        IBallisticSolver& ballistic_solver) const;

private:
    DroneMovementController movement_controller_;
    TrajectoryEstimator trajectory_estimator_;
};
