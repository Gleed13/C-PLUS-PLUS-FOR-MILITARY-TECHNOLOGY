#include <algorithm>
#include <cmath>
#include <optional>

#include "features/TrajectoryEstimator.hpp"

TrajectoryEstimator::TrajectoryEstimator(const DroneMotionProfile& motion_profile)
    : motion_profile_(motion_profile)
{
}

std::optional<float> TrajectoryEstimator::estimateArrivalTime(const DroneState& drone,
                                                              const Coord& target_position,
                                                              const DropPoint& drop_point,
                                                              const DroneConfig& config) const
{
    if (config.attackSpeed <= 0.0F || config.accelPath <= 0.0F || config.angularSpeed <= 0.0F || config.turnThreshold < 0.0F) {
        return std::nullopt;
    }

    const float acceleration = motion_profile_.acceleration();
    if (!std::isfinite(acceleration) || acceleration <= 0.0F) {
        return std::nullopt;
    }

    DroneState estimated_drone = drone;
    float total_time = 0.0F;
    float remaining_acceleration_path = motion_profile_.remainingAccelerationPath(estimated_drone.speed);
    float remaining_deceleration_path = motion_profile_.remainingDecelerationPath(estimated_drone.speed);

    if (drop_point.intermPoint.has_value()) {
        const Coord intermediate_point = drop_point.intermPoint.value();
        const float angle_to_intermediate = angleToTarget(estimated_drone.position, estimated_drone.direction, intermediate_point);

        if (std::abs(angle_to_intermediate) > config.turnThreshold) {
            total_time += estimated_drone.speed / acceleration;
            total_time += std::abs(angle_to_intermediate) / config.angularSpeed;

            estimated_drone.direction += angle_to_intermediate;
            estimated_drone.position.x += remaining_deceleration_path * std::cos(estimated_drone.direction);
            estimated_drone.position.y += remaining_deceleration_path * std::sin(estimated_drone.direction);
            remaining_acceleration_path = config.accelPath;
            remaining_deceleration_path = 0.0F;
        }
        else {
            estimated_drone.direction += angle_to_intermediate;
        }

        const float distance_to_intermediate = distance(estimated_drone.position, intermediate_point);

        if (distance_to_intermediate <= remaining_deceleration_path) {
            total_time += estimated_drone.speed / acceleration;
        }
        else if (distance_to_intermediate >= config.accelPath + remaining_acceleration_path) {
            const float distance_at_max_speed = distance_to_intermediate - remaining_acceleration_path - config.accelPath;
            total_time += (2.0F * config.attackSpeed - estimated_drone.speed) / acceleration;
            total_time += distance_at_max_speed / config.attackSpeed;
        }
        else {
            const float acceleration_path_before_deceleration =
                std::max(0.0F, (distance_to_intermediate - remaining_deceleration_path) / 2.0F);
            const float peak_speed =
                std::sqrt(estimated_drone.speed * estimated_drone.speed + 2.0F * acceleration * acceleration_path_before_deceleration);
            total_time += (2.0F * peak_speed - estimated_drone.speed) / acceleration;
        }

        estimated_drone.position = intermediate_point;
        estimated_drone.speed = 0.0F;
        estimated_drone.status = DroneStatus::Stopped;
    }

    const float angle_to_target = angleToTarget(estimated_drone.position, estimated_drone.direction, target_position);
    const float turn_threshold = estimated_drone.speed > 0.0F ? config.turnThreshold : config.turnThreshold / 2.0F;
    if (std::abs(angle_to_target) > turn_threshold) {
        total_time += std::abs(angle_to_target) / config.angularSpeed;
    }

    total_time += (config.attackSpeed - estimated_drone.speed) / acceleration;

    remaining_acceleration_path = motion_profile_.remainingAccelerationPath(estimated_drone.speed);
    const float distance_to_target = distance(estimated_drone.position, target_position);
    total_time += std::max(0.0F, distance_to_target - remaining_acceleration_path) / config.attackSpeed;

    if (!std::isfinite(total_time) || total_time < 0.0F) {
        return std::nullopt;
    }

    return total_time;
}

float TrajectoryEstimator::angleToTarget(const Coord& position, float direction, const Coord& target_position)
{
    const Coord to_target = target_position - position;
    const float target_length = std::hypot(to_target.x, to_target.y);
    if (target_length == 0.0F) {
        return 0.0F;
    }

    const Coord current_direction{std::cos(direction), std::sin(direction)};
    const Coord normalized_target{to_target.x / target_length, to_target.y / target_length};

    const float cross = current_direction.x * normalized_target.y - current_direction.y * normalized_target.x;
    const float dot = current_direction.x * normalized_target.x + current_direction.y * normalized_target.y;

    return std::atan2(cross, dot);
}

float TrajectoryEstimator::distance(const Coord& first, const Coord& second)
{
    return std::hypot(second.x - first.x, second.y - first.y);
}
