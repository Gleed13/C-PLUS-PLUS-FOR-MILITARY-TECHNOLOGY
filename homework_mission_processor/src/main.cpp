#include <string>
#include <utility>

#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"
#include "features/SimulationRecorder.hpp"
#include "strategies/StrategyFactory.hpp"

int main(int argc, char** argv)
{
    if (argc < 4 || argc > 5) {
        ERROR(
            "Usage: " << argv[0]
                      << " <config_input_file> <targets_input_file> <ballistic_table_file>"
                         " [simulation_output_file]");
        return 1;
    }

    const std::string output_path = argc == 5 ? argv[4] : "simulation.json";
    auto loader = StrategyFactory::createLoader(StrategyFactory::LoaderType::JSON);
    auto provider = StrategyFactory::createProvider(StrategyFactory::ProviderType::JSON, argv[2]);
    auto solver = StrategyFactory::createSolver(
        StrategyFactory::SolverType::TABLE,
        argv[3]);
    if (loader == nullptr ||
        provider == nullptr ||
        solver == nullptr) {
        ERROR("Failed to create processing strategies");
        return 1;
    }

    MissionProcessor processor(std::move(loader), std::move(provider), std::move(solver));
    if (!processor.init(argv[1])) {
        return 1;
    }

    const SimulationResult result = processor.run();
    const SimulationRecorder recorder;
    if (!recorder.writeJson(result, output_path)) {
        return 1;
    }

    switch (result.outcome) {
        case SimulationOutcome::TargetReached:
            LOG("Simulation reached firing range in " << result.steps.size() << " steps");
            return 0;
        case SimulationOutcome::MaxStepsReached:
            ERROR("Simulation reached the maximum step count: " << result.steps.size());
            return 2;
        case SimulationOutcome::Failed:
            ERROR("Simulation failed after " << result.steps.size() << " steps");
            return 1;
    }

    return 1;
}
