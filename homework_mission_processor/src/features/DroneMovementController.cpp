#include <algorithm>
#include <cmath>

#include "features/Logging.hpp"
#include "features/DroneMovementController.hpp"

bool DroneMovementController::update(
    DroneState& drone,
    const DropPoint& destination,
    const DroneConfig& config) const
{
    const float acceleration = calculateAcceleration(config);
    if (!std::isfinite(acceleration) || acceleration <= 0.0F) {
        ERROR("Invalid drone movement configuration");
        return false;
    }

    const Coord target_position = destination.intermPoint.value_or(destination.firePoint);
    const float target_angle = angleToTarget(drone, target_position);

    switch (drone.status) {
        case DroneStatus::Stopped:
        case DroneStatus::Turning:
            if (std::abs(target_angle) > config.turnThreshold / 2.0F) {
                drone.status = DroneStatus::Turning;
                const float direction_sign = target_angle > 0.0F ? 1.0F : -1.0F;
                drone.direction += direction_sign *
                    std::min(config.angularSpeed * config.simTimeStep, std::abs(target_angle));
            } else {
                accelerate(drone, config, acceleration);
            }
            break;

        case DroneStatus::Accelerating:
        case DroneStatus::Decelerating:
            if (std::abs(target_angle) > config.turnThreshold) {
                decelerate(drone, config, acceleration);
            } else if (destination.intermPoint.has_value()) {
                const float distance_to_intermediate = distance(drone.position, target_position);
                if (distance_to_intermediate <= remainingDecelerationPath(drone, config)) {
                    decelerate(drone, config, acceleration);
                } else {
                    accelerate(drone, config, acceleration);
                }
            } else {
                accelerate(drone, config, acceleration);
            }
            break;

        case DroneStatus::Moving:
            if (std::abs(target_angle) > config.turnThreshold) {
                decelerate(drone, config, acceleration);
            } else if (destination.intermPoint.has_value()) {
                const float distance_to_intermediate = distance(drone.position, target_position);
                if (distance_to_intermediate <= config.accelPath) {
                    decelerate(drone, config, acceleration);
                } else {
                    drone.direction += target_angle;
                }
            } else {
                drone.direction += target_angle;
            }
            break;
    }

    updatePosition(drone, config);
    return true;
}

float DroneMovementController::remainingAccelerationPath(
    const DroneState& drone,
    const DroneConfig& config) const
{
    if (config.attackSpeed <= 0.0F || config.accelPath <= 0.0F || drone.speed >= config.attackSpeed) {
        return 0.0F;
    }

    const float speed_ratio = drone.speed / config.attackSpeed;
    return config.accelPath - config.accelPath * speed_ratio * speed_ratio;
}

float DroneMovementController::remainingDecelerationPath(
    const DroneState& drone,
    const DroneConfig& config) const
{
    if (config.attackSpeed <= 0.0F || config.accelPath <= 0.0F || drone.speed <= 0.0F) {
        return 0.0F;
    }

    const float speed_ratio = drone.speed / config.attackSpeed;
    return config.accelPath * speed_ratio * speed_ratio;
}

float DroneMovementController::calculateAcceleration(const DroneConfig& config)
{
    if (config.attackSpeed <= 0.0F ||
        config.accelPath <= 0.0F ||
        config.simTimeStep <= 0.0F ||
        config.angularSpeed < 0.0F ||
        config.turnThreshold < 0.0F) {
        return 0.0F;
    }

    return config.attackSpeed * config.attackSpeed / (2.0F * config.accelPath);
}

float DroneMovementController::angleToTarget(
    const DroneState& drone,
    const Coord& target_position)
{
    const Coord to_target = target_position - drone.position;
    const float target_length = std::hypot(to_target.x, to_target.y);
    if (target_length == 0.0F) {
        return 0.0F;
    }

    const Coord drone_direction{
        std::cos(drone.direction),
        std::sin(drone.direction)
    };
    const Coord normalized_target{
        to_target.x / target_length,
        to_target.y / target_length
    };

    const float cross =
        drone_direction.x * normalized_target.y -
        drone_direction.y * normalized_target.x;
    const float dot =
        drone_direction.x * normalized_target.x +
        drone_direction.y * normalized_target.y;

    return std::atan2(cross, dot);
}

float DroneMovementController::distance(
    const Coord& first,
    const Coord& second)
{
    return std::hypot(second.x - first.x, second.y - first.y);
}

void DroneMovementController::accelerate(
    DroneState& drone,
    const DroneConfig& config,
    float acceleration)
{
    drone.speed += acceleration * config.simTimeStep;
    if (drone.speed >= config.attackSpeed) {
        drone.speed = config.attackSpeed;
        drone.status = DroneStatus::Moving;
    } else {
        drone.status = DroneStatus::Accelerating;
    }
}

void DroneMovementController::decelerate(
    DroneState& drone,
    const DroneConfig& config,
    float acceleration)
{
    drone.speed -= acceleration * config.simTimeStep;
    if (drone.speed <= 0.0F) {
        drone.speed = 0.0F;
        drone.status = DroneStatus::Stopped;
    } else {
        drone.status = DroneStatus::Decelerating;
    }
}

void DroneMovementController::updatePosition(
    DroneState& drone,
    const DroneConfig& config)
{
    drone.position.x += drone.speed * std::cos(drone.direction) * config.simTimeStep;
    drone.position.y += drone.speed * std::sin(drone.direction) * config.simTimeStep;
}
