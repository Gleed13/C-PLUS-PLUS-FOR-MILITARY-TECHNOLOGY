#pragma once

#include <optional>

#include "features/DroneMotionProfile.hpp"
#include "features/TrajectoryEstimator.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/Ammo.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/TargetSelection.hpp"

class TargetSelector final {
public:
    explicit TargetSelector(const DroneMotionProfile& motion_profile);

    std::optional<TargetSelection> select(const DroneTelemetry& droneTelemetry,
                                          float simulation_time,
                                          const DroneConfig& config,
                                          const Ammo& ammo,
                                          const ITargetProvider& target_provider,
                                          IBallisticSolver& ballistic_solver) const;

private:
    const DroneMotionProfile& motion_profile_;
    TrajectoryEstimator trajectory_estimator_;
};
