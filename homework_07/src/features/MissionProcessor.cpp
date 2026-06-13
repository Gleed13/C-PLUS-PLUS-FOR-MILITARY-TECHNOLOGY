#include "features/MissionProcessor.hpp"
#include <iostream>

MissionProcessor::MissionProcessor(IConfigLoader* config_loader, ITargetProvider* target_provider, IBallisticSolver* solver)
    : config_loader_(config_loader), target_provider_(target_provider), solver_(solver)
{
}

void MissionProcessor::init()
{
    // Load configuration
    if (!config_loader_->tryLoadConfig("data/input.txt"))
    {
        std::cerr << "Failed to load configuration\n";
        return;
    }
}

bool MissionProcessor::hasNext()
{
    return target_provider_->getTargetCount() > current_step_;
}

bool MissionProcessor::step(DropPoint* drop_point)
{
    if (!hasNext())
        return false;

    Coord* target = target_provider_->getTarget(current_step_);
    if (target == nullptr) {
        std::cerr << "Failed to get target at index " << current_step_ << '\n';
        return false;
    }

    if (!solver_->trySolve(config_loader_->getConfig(), target,config_loader_->getAmmoParams(), drop_point))
    {
        std::cerr << "Failed to solve for target at index " << current_step_ << '\n';
        return false;
    }

    ++current_step_;
    return true;
}

void MissionProcessor::reset()
{
    current_step_ = 0;
}

void MissionProcessor::changeSolver(IBallisticSolver* new_solver)
{
    solver_ = new_solver;
}