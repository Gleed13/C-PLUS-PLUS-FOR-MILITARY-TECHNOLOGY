#include <utility>

#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"
#include "strategies/StrategyFactory.hpp"

int main(int argc, char** argv)
{
    if (argc != 3) {
        ERROR("Usage: " << argv[0]
                        << " <ballistic_table_file>");
        return 1;
    }

    std::string gpio_chip_name = "/dev/gpiochip0";
    int gpio_start_line = 24;
    int gpio_drop_line = 23;
    std::shared_ptr<UartBridge> uart_bridge = std::make_shared<UartBridge>("/dev/ttyAMA3");
    auto loader = StrategyFactory::createLoader(StrategyFactory::LoaderType::JSON);
    auto provider = StrategyFactory::createProvider(StrategyFactory::ProviderType::UART, uart_bridge, argv[2]);
    auto solver = StrategyFactory::createSolver(StrategyFactory::SolverType::TABLE, argv[2]);
    auto checker_controller = StrategyFactory::createCheckerController(StrategyFactory::CheckerControllerType::MOCK, gpio_chip_name, gpio_start_line, gpio_drop_line);
    if (loader == nullptr || provider == nullptr || solver == nullptr || checker_controller == nullptr) {
        ERROR("Failed to create processing strategies");
        return 1;
    }

    MissionProcessor mission_processor(std::move(loader), std::move(provider), std::move(solver), uart_bridge, std::move(checker_controller));
    if (!mission_processor.init(argv[1])) {
        return 1;
    }

    mission_processor.start();
    mission_processor.join();

    auto result = mission_processor.getSimulationResult();
    if (!result.has_value()) {
        ERROR("Simulation has failed");
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