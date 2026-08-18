#include <cmath>
#include <cstddef>
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
                                                      const ITargetMotionProvider& target_provider,
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

    std::size_t target_count = target_provider.getTargetCount();
    for (std::size_t target_index = 0; target_index < target_count; ++target_index) {
        const auto target = target_provider.getTarget(target_index);
        if (!target.has_value()) {
            continue;
        }
        float target_pos_time_delta = simulation_time - target->timeSecSinceStart;
        auto now_target_position = target->pos + target->velocity * target_pos_time_delta;

        const auto ballistic_solution = ballistic_solver.solve(current_config, now_target_position, ammo);
        if (!ballistic_solution.has_value()) {
            continue;
        }

        DropPoint estimated_drop_point = ballistic_solution->dropPoint;
        const float distance_to_target = std::hypot(now_target_position.x - droneTelemetry.position.x, now_target_position.y - droneTelemetry.position.y);
        const float remaining_acceleration_path = motion_profile_.remainingAccelerationPath(droneTelemetry.speed);
        const bool needs_intermediate_point =
            ballistic_solution->horizontalDistance + remaining_acceleration_path - config.hitRadius * 0.5F > distance_to_target;
        if (!needs_intermediate_point) {
            estimated_drop_point.intermPoint.reset();
        }

        const auto arrival_time = trajectory_estimator_.estimateArrivalTime(droneTelemetry, now_target_position, estimated_drop_point, config);
        if (!arrival_time.has_value() || arrival_time.value() >= best_arrival_time) {
            continue;
        }

        best_arrival_time = arrival_time.value();
        best_selection = TargetSelection{
            .targetIndex = target_index, .predictedPosition = now_target_position, .targetVelocity = target->velocity, .estimatedArrivalTime = arrival_time.value()};
    }

    return best_selection;
}