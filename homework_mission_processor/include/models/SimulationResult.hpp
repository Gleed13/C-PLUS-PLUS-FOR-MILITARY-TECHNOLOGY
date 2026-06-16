#pragma once

#include <vector>

#include "SimulationStep.hpp"

enum class SimulationOutcome { TargetReached, MaxStepsReached, Failed };

struct SimulationResult {
    SimulationOutcome outcome = SimulationOutcome::Failed;
    std::vector<SimulationStep> steps;
};
