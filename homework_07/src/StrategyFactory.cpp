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

ITargetProvider* StrategyFactory::createProvider(ProviderType type, const char* param)
{
    switch (type)
    {
        case ProviderType::JSON:
            // return new JSONTargetProvider(param);
            return nullptr; // Placeholder
        default:
            return nullptr;
    }
}

IConfigLoader* StrategyFactory::createLoader(LoaderType type)
{
    switch (type)
    {
        case LoaderType::FILE:
            // return new FileConfigLoader();
            return nullptr; // Placeholder
        default:
            return nullptr;
    }
}