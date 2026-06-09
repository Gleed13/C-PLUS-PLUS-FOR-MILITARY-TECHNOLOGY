#include <cmath>
#include <limits>
#include <optional>

#include "features/TargetSelector.hpp"

std::optional<TargetSelection> TargetSelector::select(
    const DroneState& drone,
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
    current_config.startPos = drone.position;
    current_config.initialDir = drone.direction;

    for (std::size_t target_index = 0;
         target_index < target_provider.getTargetCount();
         ++target_index) {
        const auto target_position = target_provider.getPosition(
            target_index,
            simulation_time,
            config.arrayTimeStep);
        if (!target_position.has_value()) {
            continue;
        }

        const auto ballistic_solution = ballistic_solver.solve(
            current_config,
            target_position.value(),
            ammo);
        if (!ballistic_solution.has_value()) {
            continue;
        }

        DropPoint estimated_drop_point = ballistic_solution->dropPoint;
        const float distance_to_target = std::hypot(
            target_position->x - drone.position.x,
            target_position->y - drone.position.y);
        const float remaining_acceleration_path =
            movement_controller_.remainingAccelerationPath(drone, config);
        const bool needs_intermediate_point =
            ballistic_solution->horizontalDistance +
                remaining_acceleration_path -
                config.hitRadius * 0.5F >
            distance_to_target;
        if (!needs_intermediate_point) {
            estimated_drop_point.intermPoint.reset();
        }

        const auto arrival_time =
            trajectory_estimator_.estimateArrivalTime(
                drone,
                target_position.value(),
                estimated_drop_point,
                config);
        if (!arrival_time.has_value() ||
            arrival_time.value() >= best_arrival_time) {
            continue;
        }

        best_arrival_time = arrival_time.value();
        best_selection = TargetSelection{
            .targetIndex = target_index,
            .predictedPosition = target_position.value(),
            .estimatedArrivalTime = arrival_time.value()
        };
    }

    return best_selection;
}
