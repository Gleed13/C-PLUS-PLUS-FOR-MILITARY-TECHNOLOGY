#include <string>

#include "strategies/StrategyFactory.hpp"
#include "strategies/FileConfigLoader.hpp"
#include "strategies/JsonTargetProvider.hpp"
#include "strategies/AnalyticalBallisticSolver.hpp"

IConfigLoader* StrategyFactory::createLoader(LoaderType type)
{
    switch (type)
    {
        case LoaderType::FILE:
            return new FileConfigLoader();
        default:
            return nullptr;
    }
}

ITargetProvider* StrategyFactory::createProvider(ProviderType type, const std::string param)
{
    switch (type)
    {
        case ProviderType::JSON:
            return new JsonTargetProvider(param);
        default:
            return nullptr;
    }
}

IBallisticSolver* StrategyFactory::createSolver(SolverType type)
{
    switch (type)
    {
        case SolverType::ANALYTICAL:
            return new AnalyticalBallisticSolver();
        default:
            return nullptr;
    }
}