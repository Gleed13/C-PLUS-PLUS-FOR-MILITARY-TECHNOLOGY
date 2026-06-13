#include "strategies/StrategyFactory.hpp"
#include "strategies/FileConfigLoader.hpp"
#include "strategies/JsonConfigLoader.hpp"
#include "strategies/JsonTargetProvider.hpp"
#include "strategies/AnalyticalBallisticSolver.hpp"
#include "strategies/TableBallisticSolver.hpp"

std::unique_ptr<IConfigLoader> StrategyFactory::createLoader(LoaderType type)
{
    switch (type)
    {
        case LoaderType::FILE:
            return std::make_unique<FileConfigLoader>();
        case LoaderType::JSON:
            return std::make_unique<JsonConfigLoader>();
        default:
            return nullptr;
    }
}

std::unique_ptr<ITargetProvider> StrategyFactory::createProvider(ProviderType type, const std::string param)
{
    switch (type)
    {
        case ProviderType::JSON:
            return std::make_unique<JsonTargetProvider>(param);
        default:
            return nullptr;
    }
}

std::unique_ptr<IBallisticSolver> StrategyFactory::createSolver(
    SolverType type,
    const std::string& param)
{
    switch (type)
    {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalBallisticSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableBallisticSolver>(param);
        default:
            return nullptr;
    }
}
