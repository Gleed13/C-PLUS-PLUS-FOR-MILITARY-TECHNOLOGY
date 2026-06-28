#include "strategies/StrategyFactory.hpp"
#include "strategies/FileConfigLoader.hpp"
#include "strategies/JsonConfigLoader.hpp"
#include "strategies/AnalyticalBallisticSolver.hpp"
#include "strategies/TableBallisticSolver.hpp"
#include "strategies/ThreadSafeTargetProvider.hpp"

std::unique_ptr<IConfigLoader> StrategyFactory::createLoader(LoaderType type)
{
    switch (type) {
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
    switch (type) {
        case ProviderType::THREAD_SAFE_JSON:
            return std::make_unique<ThreadSafeTargetProvider>(param);
        case ProviderType::UART:
            return std::make_unique<ThreadSafeTargetProvider>(param);
        default:
            return nullptr;
    }
}

std::unique_ptr<IBallisticSolver> StrategyFactory::createSolver(SolverType type, const std::string& param)
{
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalBallisticSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableBallisticSolver>(param);
        default:
            return nullptr;
    }
}
