#pragma once

#include <memory>

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/DropPoint.hpp"

class MissionProcessor final {
public:
    MissionProcessor(
        std::unique_ptr<IConfigLoader> config_loader,
        std::unique_ptr<ITargetProvider> target_provider,
        std::unique_ptr<IBallisticSolver> solver);
    void init();
    bool hasNext();
    bool step(DropPoint* drop_point);
    void reset();
    void changeSolver(std::unique_ptr<IBallisticSolver> new_solver);

private:
    std::unique_ptr<IConfigLoader> config_loader_;
    std::unique_ptr<ITargetProvider> target_provider_;
    std::unique_ptr<IBallisticSolver> solver_;

    int current_step_ = 0;
};
