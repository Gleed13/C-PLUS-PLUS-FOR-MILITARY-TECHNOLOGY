#include <cmath>
#include <limits>
#include <optional>

#include "features/TargetSelector.hpp"

TargetSelector::TargetSelector(const DroneMotionProfile& motion_profile)
    : motion_profile_(motion_profile)
    , trajectory_estimator_(motion_profile)
{
}

std::optional<TargetSelection> TargetSelector::select(const DroneTelemetry& droneTelemetry,
                                                      float simulation_time,
                                                      const DroneConfig& config,
                                                      const Ammo& ammo,
                                                      const ITargetProvider& target_provider,
                                                      IBallisticSolver& ballistic_solver) const
{
    if (!std::isfinite(simulation_time) || simulation_time < 0.0F) {
        return std::nullopt;
    }

    std::optional<TargetSelection> best_selection;
    float best_arrival_time = std::numeric_limits<float>::max();

    DroneConfig current_config = config;
    current_config.startPos = droneTelemetry.position;
    current_config.initialDir = droneTelemetry.direction;

    for (std::size_t target_index = 0; target_index < target_provider.getTargetCount(); ++target_index) {
        const auto target_position = target_provider.getPosition(target_index, {simulation_time, config.arrayTimeStep});
        if (!target_position.has_value()) {
            continue;
        }

        const auto ballistic_solution = ballistic_solver.solve(current_config, target_position.value(), ammo);
        if (!ballistic_solution.has_value()) {
            continue;
        }

        DropPoint estimated_drop_point = ballistic_solution->dropPoint;
        const float distance_to_target = std::hypot(target_position->x - droneTelemetry.position.x, target_position->y - droneTelemetry.position.y);
        const float remaining_acceleration_path = motion_profile_.remainingAccelerationPath(droneTelemetry.speed);
        const bool needs_intermediate_point =
            ballistic_solution->horizontalDistance + remaining_acceleration_path - config.hitRadius * 0.5F > distance_to_target;
        if (!needs_intermediate_point) {
            estimated_drop_point.intermPoint.reset();
        }

        const auto arrival_time = trajectory_estimator_.estimateArrivalTime(droneTelemetry, target_position.value(), estimated_drop_point, config);
        if (!arrival_time.has_value() || arrival_time.value() >= best_arrival_time) {
            continue;
        }

        best_arrival_time = arrival_time.value();
        best_selection = TargetSelection{
            .targetIndex = target_index, .predictedPosition = target_position.value(), .estimatedArrivalTime = arrival_time.value()};
    }

    return best_selection;
}
