#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>

#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"

MissionProcessor::MissionProcessor(
    std::unique_ptr<IConfigLoader> config_loader,
    std::unique_ptr<ITargetProvider> target_provider,
    std::unique_ptr<IBallisticSolver> solver)
    : config_loader_(std::move(config_loader)),
      target_provider_(std::move(target_provider)),
      solver_(std::move(solver)),
      movement_controller_(motion_profile_),
      target_selector_(motion_profile_)
{
}

bool MissionProcessor::init()
{
    if (!config_loader_->tryLoadConfig("data/config.json"))
    {
        ERROR("Failed to load configuration");
        initialized_ = false;
        return false;
    }

    const DroneConfig* config = config_loader_->getConfig();
    if (config == nullptr || config_loader_->getAmmoParams() == nullptr) {
        ERROR("Configuration loader returned incomplete data");
        initialized_ = false;
        return false;
    }
    if (!motion_profile_.init(*config) ||
        !movement_controller_.init(*config)) {
        ERROR("Failed to initialize simulation features");
        initialized_ = false;
        return false;
    }

    drone_ = DroneState{
        .position = config->startPos,
        .direction = config->initialDir,
        .speed = 0.0F,
        .status = DroneStatus::Stopped
    };
    current_target_index_ = -1;
    initialized_ = true;
    return true;
}

SimulationResult MissionProcessor::run(std::size_t max_steps)
{
    SimulationResult result;
    if (!initialized_ || max_steps == 0) {
        ERROR("Mission processor is not initialized or max steps is zero");
        return result;
    }

    const DroneConfig& config = *config_loader_->getConfig();
    const Ammo& ammo = *config_loader_->getAmmoParams();
    result.steps.reserve(max_steps);

    const auto initial_target = target_provider_->getPosition(
        0,
        0.0F,
        config.arrayTimeStep);
    if (!initial_target.has_value()) {
        ERROR("Failed to load the initial target position");
        return result;
    }

    const auto initial_ballistic_solution = solver_->solve(
        config,
        initial_target.value(),
        ammo);
    if (!initial_ballistic_solution.has_value()) {
        ERROR("Failed to calculate initial ballistic parameters");
        return result;
    }

    for (std::size_t step_index = 0; step_index < max_steps; ++step_index) {
        const StepOutcome step_outcome = runStep(
            step_index,
            config,
            ammo,
            initial_ballistic_solution->horizontalDistance,
            result);
        if (step_outcome == StepOutcome::TargetReached) {
            result.outcome = SimulationOutcome::TargetReached;
            return result;
        }
        if (step_outcome == StepOutcome::Failed) {
            result.outcome = SimulationOutcome::Failed;
            return result;
        }
    }

    result.outcome = SimulationOutcome::MaxStepsReached;
    return result;
}

MissionProcessor::StepOutcome MissionProcessor::runStep(
    std::size_t step_index,
    const DroneConfig& config,
    const Ammo& ammo,
    float initial_horizontal_distance,
    SimulationResult& result)
{
    SimulationStep step;
    step.drone = drone_;
    step.targetIndex = current_target_index_;
    step.aimPoint =
        drone_.position +
        directionVector(drone_.direction) * initial_horizontal_distance;

    if (step_index == 0) {
        result.steps.push_back(step);
        return StepOutcome::Continue;
    }

    const float simulation_time =
        static_cast<float>(step_index) * config.simTimeStep;
    const auto selection = target_selector_.select(
        drone_,
        simulation_time,
        config,
        ammo,
        *target_provider_,
        *solver_);
    if (!selection.has_value()) {
        ERROR("Failed to select a target at simulation step " << step_index);
        return StepOutcome::Failed;
    }
    current_target_index_ = static_cast<int>(selection->targetIndex);

    DroneConfig current_config = config;
    current_config.startPos = drone_.position;
    current_config.initialDir = drone_.direction;

    const auto current_solution = solver_->solve(
        current_config,
        selection->predictedPosition,
        ammo);
    if (!current_solution.has_value()) {
        ERROR("Failed to calculate initial ballistic solution at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    const auto predicted_target = predictTargetPosition(
        selection->targetIndex,
        simulation_time,
        current_solution->fallTime);
    if (!predicted_target.has_value()) {
        ERROR("Failed to predict target position at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    auto predicted_solution = solver_->solve(
        current_config,
        predicted_target.value(),
        ammo);
    if (!predicted_solution.has_value()) {
        ERROR("Failed to calculate predicted ballistic solution at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    const float distance_to_predicted_target = std::hypot(
        predicted_target->x - drone_.position.x,
        predicted_target->y - drone_.position.y);
    const float remaining_acceleration_path =
        motion_profile_.remainingAccelerationPath(drone_.speed);
    const bool needs_intermediate_point =
        predicted_solution->horizontalDistance +
            remaining_acceleration_path -
            config.hitRadius * kHitRadiusCoefficient >
        distance_to_predicted_target;
    if (!needs_intermediate_point) {
        predicted_solution->dropPoint.intermPoint.reset();
    }

    step.dropPoint = predicted_solution->dropPoint.firePoint;
    step.predictedTarget = predicted_target.value();
    result.steps.push_back(step);

    if (isInFireRange(
            predicted_target.value(),
            predicted_solution->horizontalDistance,
            config)) {
        return StepOutcome::TargetReached;
    }

    if (!movement_controller_.update(
            drone_,
            predicted_solution->dropPoint)) {
        ERROR("Failed to update drone movement at simulation step " << step_index);
        return StepOutcome::Failed;
    }

    return StepOutcome::Continue;
}

void MissionProcessor::reset()
{
    if (!initialized_) {
        return;
    }

    const DroneConfig& config = *config_loader_->getConfig();
    drone_ = DroneState{
        .position = config.startPos,
        .direction = config.initialDir,
        .speed = 0.0F,
        .status = DroneStatus::Stopped
    };
    movement_controller_.reset();
    current_target_index_ = -1;
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> new_solver)
{
    solver_ = std::move(new_solver);
}

std::optional<Coord> MissionProcessor::predictTargetPosition(
    std::size_t target_index,
    float simulation_time,
    float fall_time) const
{
    const DroneConfig& config = *config_loader_->getConfig();
    const auto current_position = target_provider_->getPosition(
        target_index,
        simulation_time,
        config.arrayTimeStep);
    const auto previous_position = target_provider_->getPosition(
        target_index,
        std::max(0.0F, simulation_time - config.simTimeStep),
        config.arrayTimeStep);
    if (!current_position.has_value() || !previous_position.has_value()) {
        return std::nullopt;
    }

    const Coord target_velocity =
        (current_position.value() - previous_position.value()) /
        config.simTimeStep;
    return current_position.value() + target_velocity * fall_time;
}

bool MissionProcessor::isInFireRange(
    const Coord& predicted_target,
    float horizontal_distance,
    const DroneConfig& config) const
{
    if (drone_.status != DroneStatus::Moving) {
        return false;
    }

    const Coord predicted_hit =
        drone_.position +
        directionVector(drone_.direction) * horizontal_distance;
    const float distance_to_target = std::hypot(
        predicted_target.x - predicted_hit.x,
        predicted_target.y - predicted_hit.y);

    return distance_to_target <=
        config.hitRadius * kHitRadiusCoefficient;
}

Coord MissionProcessor::directionVector(float direction)
{
    return {
        std::cos(direction),
        std::sin(direction)
    };
}
