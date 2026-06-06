#pragma once

#include <string>

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

class StrategyFactory {
public:
enum class LoaderType { FILE, JSON };
enum class ProviderType { JSON };
enum class SolverType { ANALYTICAL };

    static IConfigLoader* createLoader(LoaderType type);
    static ITargetProvider* createProvider(ProviderType type, const std::string param);
    static IBallisticSolver* createSolver(SolverType type);
};