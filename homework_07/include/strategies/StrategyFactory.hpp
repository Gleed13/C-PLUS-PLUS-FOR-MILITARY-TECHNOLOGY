#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

class StrategyFactory {
public:
enum class LoaderType { FILE };
enum class ProviderType { JSON };
enum class SolverType { ANALYTICAL };

    static IConfigLoader* createLoader(LoaderType type);
    static ITargetProvider* createProvider(ProviderType type, const char* param);
    static IBallisticSolver* createSolver(SolverType type);
};