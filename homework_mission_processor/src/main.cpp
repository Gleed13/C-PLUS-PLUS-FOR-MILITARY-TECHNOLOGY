#include <latch>
#include <string>
#include <utility>

#include "features/DronePhysics.hpp"
#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"
#include "features/SimulationRecorder.hpp"
#include "strategies/StrategyFactory.hpp"
#include "strategies/ThreadSafeTargetProvider.hpp"

int main(int argc, char** argv)
{
    if (argc < 4 || argc > 5) {
        ERROR("Usage: " << argv[0]
                        << " <config_input_file> <targets_input_file> <ballistic_table_file>"
                           " [simulation_output_file]");
        return 1;
    }

    const std::string output_path = argc == 5 ? argv[4] : "simulation.json";
    auto loader = StrategyFactory::createLoader(StrategyFactory::LoaderType::JSON);
    auto provider = StrategyFactory::createProvider(StrategyFactory::ProviderType::THREAD_SAFE_JSON, argv[2]);
    auto solver = StrategyFactory::createSolver(StrategyFactory::SolverType::TABLE, argv[3]);
    if (loader == nullptr || provider == nullptr || solver == nullptr) {
        ERROR("Failed to create processing strategies");
        return 1;
    }

    MissionProcessor mission_processor(std::move(loader), std::move(provider), std::move(solver));
    if (!mission_processor.init(argv[1])) {
        return 1;
    }

    int number_of_services = 2;
    ThreadSafeTargetProvider* thread_safe_provider = dynamic_cast<ThreadSafeTargetProvider*>(provider.get());
    if (thread_safe_provider != nullptr) {
        ++number_of_services;
    }
    DronePhysics& drone_physics = mission_processor.getDronePhysics();

    auto ready_latch = std::make_shared<std::latch>(number_of_services);
    auto start_gate = std::make_shared<std::latch>(1);

    if (thread_safe_provider != nullptr) {
        thread_safe_provider->start(ready_latch, start_gate);
    }
    drone_physics.start(ready_latch, start_gate);
    mission_processor.start(ready_latch, start_gate);

    ready_latch->wait();     // wait until all workers are ready
    start_gate->count_down(); // release them together

    mission_processor.join();

    auto result = mission_processor.getSimulationResult();
    if (!result.has_value()) {
        ERROR("Simulation has failed, no result will be written to the output file");
        return 1;
    }
    const SimulationRecorder recorder;
    if (!recorder.writeJson(result.value(), output_path)) {
        return 1;
    }

    switch (result.value().outcome) {
        case SimulationOutcome::TargetReached:
            LOG("Simulation reached firing range in " << result.value().steps.size() << " steps");
            return 0;
        case SimulationOutcome::MaxStepsReached:
            ERROR("Simulation reached the maximum step count: " << result.value().steps.size());
            return 2;
        case SimulationOutcome::Failed:
            ERROR("Simulation failed after " << result.value().steps.size() << " steps");
            return 1;
    }

    return 1;
}
