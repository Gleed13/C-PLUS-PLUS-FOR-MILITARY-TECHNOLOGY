#include <cmath>
#include <memory>
#include <utility>

#include "features/Logging.hpp"
#include "features/DroneMovementController.hpp"

DroneMovementController::DroneMovementController(
    const DroneMotionProfile& motion_profile)
    : motion_profile_(motion_profile)
{
}

bool DroneMovementController::init(const DroneConfig& config)
{
    if (config.attackSpeed <= 0.0F ||
        config.accelPath <= 0.0F ||
        config.simTimeStep <= 0.0F ||
        config.angularSpeed < 0.0F ||
        config.turnThreshold < 0.0F) {
        ERROR("Invalid drone movement configuration");
        config_ = nullptr;
        state_.reset();
        return false;
    }

    if (motion_profile_.acceleration() <= 0.0F) {
        ERROR("Drone motion profile is not initialized");
        config_ = nullptr;
        state_.reset();
        return false;
    }

    config_ = &config;
    reset();
    return true;
}

void DroneMovementController::reset()
{
    state_ = std::make_unique<StateStopped>();
}

bool DroneMovementController::update(
    DroneState& drone,
    const DropPoint& destination)
{
    if (config_ == nullptr || state_ == nullptr) {
        ERROR("Drone movement controller is not initialized");
        return false;
    }
    const DroneConfig& config = *config_;

    const Coord target_position = destination.intermPoint.value_or(destination.firePoint);
    const float target_angle = angleToTarget(drone, target_position);
    DroneMovementContext context{
        .drone = drone,
        .config = config,
        .motionProfile = motion_profile_,
        .destination = destination,
        .targetPosition = target_position,
        .targetAngle = target_angle
    };
    auto next_state = state_->execute(context);
    if (next_state != nullptr) {
        state_ = std::move(next_state);
    }

    updatePosition(drone, config);
    return true;
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

void DroneMovementController::updatePosition(
    DroneState& drone,
    const DroneConfig& config)
{
    drone.position.x += drone.speed * std::cos(drone.direction) * config.simTimeStep;
    drone.position.y += drone.speed * std::sin(drone.direction) * config.simTimeStep;
}
