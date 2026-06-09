#pragma once

#include <cstddef>
#include <memory>

#include "features/DroneMovementController.hpp"
#include "features/TargetSelector.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/DroneState.hpp"
#include "models/SimulationResult.hpp"

class MissionProcessor final {
public:
    MissionProcessor(
        std::unique_ptr<IConfigLoader> config_loader,
        std::unique_ptr<ITargetProvider> target_provider,
        std::unique_ptr<IBallisticSolver> solver);
    bool init();
    SimulationResult run(std::size_t max_steps = 10000);
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> new_solver);

private:
    static constexpr float kHitRadiusCoefficient = 0.5F;

    std::unique_ptr<IConfigLoader> config_loader_;
    std::unique_ptr<ITargetProvider> target_provider_;
    std::unique_ptr<IBallisticSolver> solver_;

    DroneMovementController movement_controller_;
    TargetSelector target_selector_;
    DroneState drone_;
    bool initialized_ = false;

    std::optional<Coord> predictTargetPosition(
        std::size_t target_index,
        float simulation_time,
        float fall_time) const;
    bool isInFireRange(
        const Coord& predicted_target,
        float horizontal_distance,
        const DroneConfig& config) const;
    static Coord directionVector(float direction);
};
