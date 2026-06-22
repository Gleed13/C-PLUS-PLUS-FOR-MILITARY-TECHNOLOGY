#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>

#include "abstractions/IResettable.hpp"
#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneTelemetry.hpp"

MissionProcessor::MissionProcessor(std::unique_ptr<IConfigLoader> config_loader,
                                   std::unique_ptr<ITargetProvider> target_provider,
                                   std::unique_ptr<IBallisticSolver> solver)
    : config_loader_(std::move(config_loader))
    , target_provider_(std::move(target_provider))
    , solver_(std::move(solver))
    , drone_physics_(motion_profile_)
    , target_selector_(motion_profile_)
{
}

bool MissionProcessor::init(const std::string& config_path, std::size_t max_steps)
{
    max_steps_ = max_steps;

    if (!config_loader_->tryLoadConfig(config_path)) {
        ERROR("Failed to load configuration");
        initialized_ = false;
        return false;
    }

    const std::shared_ptr<DroneConfig> config = config_loader_->getConfig();
    if (config == nullptr || config_loader_->getAmmoParams() == nullptr) {
        ERROR("Configuration loader returned incomplete data");
        initialized_ = false;
        return false;
    }
    if (!motion_profile_.init(*config) || !drone_physics_.init(config)) {
        ERROR("Failed to initialize simulation features");
        initialized_ = false;
        return false;
    }

    current_target_index_ = -1;
    initialized_ = true;
    return true;
}

MissionProcessor::StepOutcome MissionProcessor::runStep(
    std::size_t step_index, const DroneConfig& config, const Ammo& ammo, float initial_horizontal_distance)
{
    auto drone_telemetry = drone_physics_.getTelemetry();
    SimulationStep step;
    step.droneTelemetry = drone_telemetry;
    step.targetIndex = current_target_index_;
    step.aimPoint = drone_telemetry.position + directionVector(drone_telemetry.direction) * initial_horizontal_distance;

    if (step_index == 0) {
        simulation_result_.value().steps.push_back(step);
        return StepOutcome::Continue;
    }

    const float simulation_time = static_cast<float>(step_index) * config.simTimeStep;
    const auto selection = target_selector_.select(drone_telemetry, simulation_time, config, ammo, *target_provider_, *solver_);
    if (!selection.has_value()) {
        ERROR("Failed to select a target at simulation step " << step_index);
        return StepOutcome::Failed;
    }
    current_target_index_ = static_cast<int>(selection->targetIndex);

    DroneConfig current_config = config;
    current_config.startPos = drone_telemetry.position;
    current_config.initialDir = drone_telemetry.direction;

    const auto current_solution = solver_->solve(current_config, selection->predictedPosition, ammo);
    if (!current_solution.has_value()) {
        ERROR("Failed to calculate initial ballistic solution at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    const auto predicted_target = predictTargetPosition(selection->targetIndex, simulation_time, current_solution->fallTime);
    if (!predicted_target.has_value()) {
        ERROR("Failed to predict target position at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    auto predicted_solution = solver_->solve(current_config, predicted_target.value(), ammo);
    if (!predicted_solution.has_value()) {
        ERROR("Failed to calculate predicted ballistic solution at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    const float distance_to_predicted_target = std::hypot(predicted_target->x - drone_telemetry.position.x, predicted_target->y - drone_telemetry.position.y);
    const float remaining_acceleration_path = motion_profile_.remainingAccelerationPath(drone_telemetry.speed);
    const bool needs_intermediate_point =
        predicted_solution->horizontalDistance + remaining_acceleration_path - config.hitRadius * kHitRadiusCoefficient >
        distance_to_predicted_target;
    if (!needs_intermediate_point) {
        predicted_solution->dropPoint.intermPoint.reset();
    }

    step.dropPoint = predicted_solution->dropPoint.firePoint;
    step.predictedTarget = predicted_target.value();
    simulation_result_.value().steps.push_back(step);

    if (isInFireRange(drone_telemetry, predicted_target.value(), predicted_solution->horizontalDistance, config)) {
        return StepOutcome::TargetReached;
    }

    if (!drone_physics_.updateDestination(&predicted_solution->dropPoint)) {
        ERROR("Failed to update drone movement at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    return StepOutcome::Continue;
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> new_solver)
{
    solver_ = std::move(new_solver);
}

void MissionProcessor::reset()
{
    if (!initialized_) {
        return;
    }

    if (auto* resettable = dynamic_cast<IResettable*>(target_provider_.get())) {
        resettable->reset();
    }
    drone_physics_.reset();
    current_target_index_ = -1;
}

std::optional<Coord> MissionProcessor::predictTargetPosition(std::size_t target_index, float simulation_time, float fall_time) const
{
    const DroneConfig& config = *config_loader_->getConfig();
    const auto current_position = target_provider_->getPosition(target_index, {simulation_time, config.arrayTimeStep});
    const auto previous_position =
        target_provider_->getPosition(target_index, {std::max(0.0F, simulation_time - config.simTimeStep), config.arrayTimeStep});
    if (!current_position.has_value() || !previous_position.has_value()) {
        return std::nullopt;
    }

    const Coord target_velocity = (current_position.value() - previous_position.value()) / config.simTimeStep;
    return current_position.value() + target_velocity * fall_time;
}

bool MissionProcessor::isInFireRange(const DroneTelemetry& drone_telemetry, const Coord& predicted_target, float horizontal_distance, const DroneConfig& config) const
{
    if (drone_telemetry.status != DroneStatus::Moving) {
        return false;
    }

    const Coord predicted_hit = drone_telemetry.position + directionVector(drone_telemetry.direction) * horizontal_distance;
    const float distance_to_target = std::hypot(predicted_target.x - predicted_hit.x, predicted_target.y - predicted_hit.y);

    return distance_to_target <= config.hitRadius * kHitRadiusCoefficient;
}

Coord MissionProcessor::directionVector(float direction)
{
    return {std::cos(direction), std::sin(direction)};
}

void MissionProcessor::start()
{
    if (!initialized_ || max_steps_ == 0) {
        ERROR("Mission processor is not initialized or max steps is zero");
        simulation_result_ = std::nullopt;
        return;
    }

    BackgroundService::start();
}

void MissionProcessor::run()
{
    auto config = config_loader_->getConfig();
    const Ammo& ammo = *config_loader_->getAmmoParams();
    const auto initial_target = target_provider_->getPosition(0, {0.0F, config->arrayTimeStep});
    if (!initial_target.has_value()) {
        ERROR("Failed to load the initial target position");
        return;
    }
    const auto initial_ballistic_solution = solver_->solve(*config, initial_target.value(), ammo);
    if (!initial_ballistic_solution.has_value()) {
        ERROR("Failed to calculate initial ballistic parameters");
        return;
    }
    simulation_result_ = SimulationResult{};
    simulation_result_.value().steps.reserve(max_steps_);


    const auto interval = std::chrono::milliseconds(
        static_cast<int>(config->simTimeStep * 1000.0F / config->timeScale));
    std::size_t step_index = 0;

    while (!stop_requested() || step_index >= max_steps_) {
        auto step_start_time = std::chrono::steady_clock::now();

        const StepOutcome step_outcome = runStep(step_index, *config, ammo, initial_ballistic_solution->horizontalDistance);
        if (step_outcome == StepOutcome::TargetReached) {
            simulation_result_.value().outcome = SimulationOutcome::TargetReached;
            return;
        }
        if (step_outcome == StepOutcome::Failed) {
            simulation_result_.value().outcome = SimulationOutcome::Failed;
            return;
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

    simulation_result_.value().outcome = SimulationOutcome::MaxStepsReached;
}