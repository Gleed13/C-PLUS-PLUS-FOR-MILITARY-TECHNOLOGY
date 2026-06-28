#pragma once

#include <memory>
#include <string>

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

class StrategyFactory {
public:
    enum class LoaderType { FILE, JSON };
    enum class ProviderType { JSON, THREAD_SAFE_JSON, UART };
    enum class SolverType { ANALYTICAL, TABLE };

    static std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
    static std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string param = {});
    static std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const std::string& param = "");
};
