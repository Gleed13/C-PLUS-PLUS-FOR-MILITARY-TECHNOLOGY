#include "strategies/StrategyFactory.hpp"
#include <memory>
#include "features/GpioController.hpp"
#include "strategies/FileConfigLoader.hpp"
#include "strategies/JsonConfigLoader.hpp"
#include "strategies/AnalyticalBallisticSolver.hpp"
#include "strategies/MockCheckerController.hpp"
#include "strategies/TableBallisticSolver.hpp"
#include "strategies/ThreadSafeTargetProvider.hpp"
#include "strategies/UartTargetProvider.hpp"

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

std::unique_ptr<ITargetMotionProvider> StrategyFactory::createProvider(ProviderType type, std::shared_ptr<UartBridge> uart_bridge, const std::string param)
{
    switch (type) {
        case ProviderType::THREAD_SAFE_JSON:
            return std::make_unique<ThreadSafeTargetProvider>(param);
        case ProviderType::UART:
            return std::make_unique<UartTargetProvider>(uart_bridge);
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

std::unique_ptr<ICheckerController> StrategyFactory::createCheckerController(CheckerControllerType type, const std::string& chip_path, const int start_line, const int drop_line)
{
    switch (type) {
        case CheckerControllerType::GPIO:
            return std::make_unique<GpioController>(chip_path, start_line, drop_line);
        case CheckerControllerType::MOCK:
            return std::make_unique<MockCheckerController>(chip_path, start_line, drop_line);
        default:
            return nullptr;
    }
}