#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "abstractions/BackgroundService.hpp"
#include "features/DroneMotionProfile.hpp"
#include "features/TargetSelector.hpp"
#include "features/UartBridge.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ICheckerController.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetMotionProvider.hpp"
#include "models/SimulationResult.hpp"

class MissionProcessor final : public BackgroundService {
public:
    MissionProcessor(std::unique_ptr<IConfigLoader> config_loader,
                     std::unique_ptr<ITargetMotionProvider> target_provider,
                     std::unique_ptr<IBallisticSolver> solver,
                     std::shared_ptr<UartBridge> uart_bridge,
                     std::unique_ptr<ICheckerController> checker_controller);
    bool init(const std::string& config_path, std::size_t max_steps = 10000);
    void changeSolver(std::unique_ptr<IBallisticSolver> new_solver);
    void start() override;
    void reset();
    std::optional<SimulationResult> getSimulationResult();

private:
    static constexpr float kHitRadiusCoefficient = 0.5F;

    enum class StepOutcome { Continue, TargetReached, Failed };

    std::unique_ptr<IConfigLoader> config_loader_;
    std::unique_ptr<ITargetMotionProvider> target_provider_;
    std::unique_ptr<IBallisticSolver> solver_;
    std::shared_ptr<UartBridge> uart_bridge_;
    std::unique_ptr<ICheckerController> checker_controller_;

    DroneMotionProfile motion_profile_;
    TargetSelector target_selector_;
    bool initialized_ = false;
    int current_target_index_ = -1;
    std::size_t max_steps_ = 10000;
    std::optional<SimulationResult> simulation_result_ = std::nullopt;

    void fixTelemetryStateIfNeeded(DroneTelemetry& telemetry, const DroneConfig& config);
    StepOutcome runStep(
        std::size_t step_index, const DroneConfig& config, const Ammo& ammo, float initial_horizontal_distance);
    bool isInFireRange(const DroneTelemetry& drone_telemetry, const Coord& predicted_target, float horizontal_distance, const DroneConfig& config) const;
    static Coord directionVector(float direction);

    void run() override;
};
