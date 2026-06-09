#include <memory>
#include <utility>

#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"

MissionProcessor::MissionProcessor(
    std::unique_ptr<IConfigLoader> config_loader,
    std::unique_ptr<ITargetProvider> target_provider,
    std::unique_ptr<IBallisticSolver> solver)
    : config_loader_(std::move(config_loader)),
      target_provider_(std::move(target_provider)),
      solver_(std::move(solver))
{
}

void MissionProcessor::init()
{
    // Load configuration
    if (!config_loader_->tryLoadConfig("data/config.json"))
    {
        ERROR("Failed to load configuration");
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
        ERROR("Failed to get target at index " << current_step_);
        return false;
    }

    if (!solver_->trySolve(config_loader_->getConfig(), target,config_loader_->getAmmoParams(), drop_point))
    {
        ERROR("Failed to solve for target at index " << current_step_);
        return false;
    }

    ++current_step_;
    return true;
}

void MissionProcessor::reset()
{
    current_step_ = 0;
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> new_solver)
{
    solver_ = std::move(new_solver);
}
