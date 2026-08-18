#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "abstractions/BackgroundService.hpp"
#include "features/DroneMovementStates.hpp"
#include "features/Logging.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/DropPoint.hpp"
#include "utils/ControlUtils.hpp"
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
        return false;
    }

    if (motion_profile_.acceleration() <= 0.0F) {
        ERROR("Drone motion profile is not initialized");
        config_ = nullptr;
        return false;
    }

    config_ = config;
    state_ = std::make_unique<StateStopped>();
    return true;
}

bool DronePhysics::updateDestination(const DropPoint* destination)
{
    if (destination == nullptr) {
        ERROR("Destination is null");
        return false;
    }

    std::lock_guard<std::mutex> lock(destination_mutex_);
    current_destination_ = *destination;
    return true;
}

void DronePhysics::updateTelemetry(DroneTelemetry telemetry)
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    drone_telemetry_ = telemetry;
}

void DronePhysics::start(std::shared_ptr<std::latch> ready_latch, std::shared_ptr<std::latch> start_gate)
{
    if (config_ == nullptr || state_ == nullptr) {
        ERROR("Drone movement controller is not initialized");
        return;
    }

    BackgroundService::start(ready_latch, start_gate);
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

void DronePhysics::updatePosition()
{
    drone_.position.x += drone_.speed * std::cos(drone_.direction) * config_->physicsTimeStep;
    drone_.position.y += drone_.speed * std::sin(drone_.direction) * config_->physicsTimeStep;
}

std::optional<DroneMovementContext> DronePhysics::createMovementContext()
{
    std::lock_guard<std::mutex> lock(destination_mutex_);
    if (!current_destination_.has_value()) {
        return std::nullopt;
    }

    const Coord target_position = current_destination_->intermPoint.value_or(current_destination_->firePoint);
    const float target_angle = ControlUtils::angleToTarget(target_position, drone_);
    DroneMovementContext context{.drone = drone_,
                                 .config = *config_,
                                 .motionProfile = motion_profile_,
                                 .destination = current_destination_.value(),
                                 .targetPosition = target_position,
                                 .targetAngle = target_angle};
    return context;
}

void DronePhysics::run()
{
    const auto interval = std::chrono::milliseconds(
        static_cast<int>(config_->physicsTimeStep * 1000.0F / config_->timeScale));
    int physics_steps = -1; // -1 to ignore first update before first delay

    while (!stop_requested()) {
        auto step_start_time = std::chrono::steady_clock::now();

        auto context = createMovementContext();
        if (context.has_value()) {
            auto next_state = state_->execute(context.value());
            if (next_state != nullptr) {
                state_ = std::move(next_state);
            }
            updatePosition();
            ++physics_steps;
            auto telemetry =
                DroneTelemetry{.position = drone_.position,
                               .direction = drone_.direction,
                               .speed = drone_.speed,
                               .status = drone_.status,
                               .timeSecSinceStart = physics_steps * config_->physicsTimeStep * config_->timeScale
                };
            updateTelemetry(telemetry);
        }

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