#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/DropPoint.hpp"

class MissionProcessor final {
public:
    MissionProcessor(IConfigLoader* config_loader, ITargetProvider* target_provider, IBallisticSolver* solver);
    void init();
    bool hasNext();
    bool step(DropPoint* drop_point);
    void reset();
    void changeSolver(IBallisticSolver* new_solver);

private:
    IConfigLoader* config_loader_;
    ITargetProvider* target_provider_;
    IBallisticSolver* solver_;

    int current_step_ = 0;
};