#include <utility>

#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"
#include "strategies/StrategyFactory.hpp"

int main(int argc, char** argv)
{
    if (argc != 2) {
        ERROR("Usage: " << argv[0] << " <targets_input_file>");
        return 1;
    }

    auto loader = StrategyFactory::createLoader(StrategyFactory::LoaderType::JSON);
    auto provider = StrategyFactory::createProvider(StrategyFactory::ProviderType::JSON, argv[1]);
    auto solver = StrategyFactory::createSolver(StrategyFactory::SolverType::ANALYTICAL);

    MissionProcessor processor(std::move(loader), std::move(provider), std::move(solver));
    processor.init();

    // Process all targets
    while (processor.hasNext()) {
        DropPoint drop_point;
        if (processor.step(&drop_point)) {
            if (drop_point.intermPoint.has_value()) {
                LOG("Calculated intermediate point: (" << drop_point.intermPoint->x << ", " << drop_point.intermPoint->y << ")");
            }
            LOG("Calculated fire point: (" << drop_point.firePoint.x << ", " << drop_point.firePoint.y << ")");
        } else {
            ERROR("Failed to calculate drop point for current target");
        }
    }
}
