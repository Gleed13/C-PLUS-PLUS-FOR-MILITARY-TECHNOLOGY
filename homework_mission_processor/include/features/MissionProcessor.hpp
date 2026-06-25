#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "abstractions/BackgroundService.hpp"
#include "features/DroneMotionProfile.hpp"
#include "features/DronePhysics.hpp"
#include "features/TargetSelector.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/SimulationResult.hpp"

class MissionProcessor final : public BackgroundService {
public:
    MissionProcessor(std::unique_ptr<IConfigLoader> config_loader,
                     std::unique_ptr<ITargetProvider> target_provider,
                     std::unique_ptr<IBallisticSolver> solver);
    bool init(const std::string& config_path, std::size_t max_steps = 10000);
    void changeSolver(std::unique_ptr<IBallisticSolver> new_solver);
    void start(std::shared_ptr<std::latch> ready_latch, std::shared_ptr<std::latch> start_gate) override;
    void reset();
    std::optional<SimulationResult> getSimulationResult();

public:
    DronePhysics& getDronePhysics() { return drone_physics_; }

private:
    static constexpr float kHitRadiusCoefficient = 0.5F;

    enum class StepOutcome { Continue, TargetReached, Failed };

    std::unique_ptr<IConfigLoader> config_loader_;
    std::unique_ptr<ITargetProvider> target_provider_;
    std::unique_ptr<IBallisticSolver> solver_;

    DroneMotionProfile motion_profile_;
    DronePhysics drone_physics_;
    TargetSelector target_selector_;
    bool initialized_ = false;
    int current_target_index_ = -1;
    std::size_t max_steps_ = 10000;
    std::optional<SimulationResult> simulation_result_ = std::nullopt;

    StepOutcome runStep(
        std::size_t step_index, const DroneConfig& config, const Ammo& ammo, float initial_horizontal_distance);
    bool isInFireRange(const DroneTelemetry& drone_telemetry, const Coord& predicted_target, float horizontal_distance, const DroneConfig& config) const;
    static Coord directionVector(float direction);

    void run() override;
};
