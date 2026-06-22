#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include "abstractions/BackgroundService.hpp"
#include "features/Logging.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/DropPoint.hpp"
#include "features/DronePhysics.hpp"

DronePhysics::DronePhysics(const DroneMotionProfile& motion_profile)
    : motion_profile_(motion_profile)
{
}

bool DronePhysics::init(std::shared_ptr<DroneConfig> config)
{
    if (config->attackSpeed <= 0.0F || config->accelPath <= 0.0F || config->simTimeStep <= 0.0F || config->angularSpeed < 0.0F ||
        config->turnThreshold < 0.0F) {
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

    config_ = config;
    reset();
    return true;
}

bool DronePhysics::updateDestination(const DropPoint* destination)
{
    if (destination == nullptr) {
        ERROR("Destination is null");
        return false;
    }

    current_destination_ = std::make_unique<DropPoint>(*destination);
    return true;
}

void DronePhysics::updateTelemetry(DroneTelemetry telemetry)
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    drone_telemetry_ = telemetry;
}

void DronePhysics::reset()
{
    stop();
    drone_ = DroneState{.position = config_->startPos, .direction = config_->initialDir, .speed = 0.0F, .status = DroneStatus::Stopped};
    current_destination_.reset();
    state_.reset();
    auto telemetry = DroneTelemetry{
        .position = drone_.position,
        .direction = drone_.direction,
        .speed = drone_.speed,
        .status = drone_.status,
        .timeSecSinceStart = 0.0F};
    updateTelemetry(telemetry);
}

DroneTelemetry DronePhysics::getTelemetry()
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return drone_telemetry_;
}

float DronePhysics::angleToTarget(const Coord& target_position)
{
    const Coord to_target = target_position - drone_.position;
    const float target_length = std::hypot(to_target.x, to_target.y);
    if (target_length == 0.0F) {
        return 0.0F;
    }

    const Coord drone_direction{std::cos(drone_.direction), std::sin(drone_.direction)};
    const Coord normalized_target{to_target.x / target_length, to_target.y / target_length};

    const float cross = drone_direction.x * normalized_target.y - drone_direction.y * normalized_target.x;
    const float dot = drone_direction.x * normalized_target.x + drone_direction.y * normalized_target.y;

    return std::atan2(cross, dot);
}

void DronePhysics::updatePosition()
{
    drone_.position.x += drone_.speed * std::cos(drone_.direction) * config_->physicsTimeStep;
    drone_.position.y += drone_.speed * std::sin(drone_.direction) * config_->physicsTimeStep;
}

void DronePhysics::start()
{
    if (config_ == nullptr || state_ == nullptr) {
        ERROR("Drone movement controller is not initialized");
        return;
    }

    BackgroundService::start();
}

void DronePhysics::run()
{
    const auto interval = std::chrono::milliseconds(
        static_cast<int>(config_->physicsTimeStep * 1000.0F / config_->timeScale));
    auto start_time = std::chrono::steady_clock::now();

    while (!stop_requested()) {
        auto step_start_time = std::chrono::steady_clock::now();

        const Coord target_position = current_destination_->intermPoint.value_or(current_destination_->firePoint);
        const float target_angle = angleToTarget(target_position);
        DroneMovementContext context{.drone = drone_,
                                     .config = *config_,
                                     .motionProfile = motion_profile_,
                                     .destination = *current_destination_,
                                     .targetPosition = target_position,
                                     .targetAngle = target_angle};
        auto next_state = state_->execute(context);
        if (next_state != nullptr) {
            state_ = std::move(next_state);
        }
        updatePosition();
        auto current_time = std::chrono::steady_clock::now();
        auto telemetry = DroneTelemetry{
            .position = drone_.position,
            .direction = drone_.direction,
            .speed = drone_.speed,
            .status = drone_.status,
            .timeSecSinceStart = std::chrono::duration<float>(current_time - start_time).count()};
        updateTelemetry(telemetry);

        auto step_end_time = std::chrono::steady_clock::now();
        auto step_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(step_end_time - step_start_time);
        auto remaining_sleep_time = interval - step_elapsed_time;
        if (remaining_sleep_time <= std::chrono::nanoseconds(0)) {
            continue;
        }

        bool stopped = wait_for(remaining_sleep_time);
        if (stopped) {
            break;
        }
    }
}