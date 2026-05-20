#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

class StrategyFactory {
    enum class SolverType { ANALYTICAL };
    enum class ProviderType { JSON };
    enum class LoaderType { FILE };

    IBallisticSolver* createSolver(SolverType type);
    ITargetProvider* createProvider(ProviderType type, const char* param);
    IConfigLoader* createLoader(LoaderType type);
};