#include <iostream>

#include "features/MissionProcessor.hpp"
#include "strategies/StrategyFactory.hpp"

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <targets_input_file>\n";
        return 1;
    }

    IConfigLoader* loader = StrategyFactory::createLoader(StrategyFactory::LoaderType::FILE);
    ITargetProvider* provider = StrategyFactory::createProvider(StrategyFactory::ProviderType::JSON, argv[1]);
    IBallisticSolver* solver = StrategyFactory::createSolver(StrategyFactory::SolverType::ANALYTICAL);

    MissionProcessor processor(loader, provider, solver);
    processor.init();

    // Process all targets
    while (processor.hasNext()) {
        DropPoint drop_point;
        if (processor.step(&drop_point)) {
            if (drop_point.intermPoint.has_value()) {
                std::cout << "Calculated intermediate point: (" << drop_point.intermPoint->x << ", " << drop_point.intermPoint->y << ")\n";
            }
            std::cout << "Calculated fire point: (" << drop_point.firePoint.x << ", " << drop_point.firePoint.y << ")\n";
        } else {
            std::cerr << "Failed to calculate drop point for current target\n";
        }
    }

    delete loader;
    loader = nullptr;
    delete provider;
    provider = nullptr;
    delete solver;
    solver = nullptr;
}