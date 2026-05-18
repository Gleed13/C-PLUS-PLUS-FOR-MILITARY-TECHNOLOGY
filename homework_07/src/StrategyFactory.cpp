#include "StrategyFactory.hpp"

IBallisticSolver* StrategyFactory::createSolver(SolverType type)
{
    switch (type)
    {
        case SolverType::ANALYTICAL:
            // return new AnalyticalBallisticSolver();
            return nullptr; // Placeholder
        default:
            return nullptr;
    }
}