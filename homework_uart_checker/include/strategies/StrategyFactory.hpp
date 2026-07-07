#pragma once

#include <memory>
#include <string>

#include "features/UartBridge.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ICheckerController.hpp"
#include "interfaces/ITargetMotionProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

class StrategyFactory {
public:
    enum class LoaderType { FILE, JSON };
    enum class ProviderType { JSON, THREAD_SAFE_JSON, UART };
    enum class SolverType { ANALYTICAL, TABLE };
    enum class CheckerControllerType { GPIO, MOCK };

    static std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
    static std::unique_ptr<ITargetMotionProvider> createProvider(ProviderType type, std::shared_ptr<UartBridge> uart_bridge, const std::optional<std::string> param = {});
    static std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const std::string& param = "");
    static std::unique_ptr<ICheckerController> createCheckerController(CheckerControllerType type, const std::string& chip_path, const unsigned start_line, const unsigned drop_line);
};
