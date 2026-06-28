#include <unistd.h>
#include <drone_link.h>

int openUart(const char* dev);

#include <utility>

#include "features/DronePhysics.hpp"
#include "features/Logging.hpp"
#include "features/MissionProcessor.hpp"
#include "strategies/StrategyFactory.hpp"
#include "strategies/ThreadSafeTargetProvider.hpp"

int main(int argc, char** argv)
{
    if (argc != 3) {
        ERROR("Usage: " << argv[0]
                        << " <ballistic_table_file>");
        return 1;
    }

    auto loader = StrategyFactory::createLoader(StrategyFactory::LoaderType::JSON);
    auto provider = StrategyFactory::createProvider(StrategyFactory::ProviderType::UART);
    auto solver = StrategyFactory::createSolver(StrategyFactory::SolverType::TABLE, argv[2]);
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

int main1(int argc, char** argv)
{
    auto fd = openUart("/dev/ttyAMA3");

    dlink::Parser parser; // тримає стан між викликами
    uint8_t buf[256];
    while (true) {
        int n = read(fd, buf, sizeof(buf)); // прочитати доступні байти
        uint8_t type, len, payload[260];
        for (int i = 0; i < n; i++) {
            if (parser.feed(buf[i], type, payload, len)) { // зібрався цілий кадр
                if (type == dlink::PKT_TELEMETRY) {
                    dlink::Telemetry telemetry;
                    memcpy(&telemetry, payload, sizeof telemetry);
                    // отримані дані телеметрії
                }
                if (type == dlink::PKT_TARGET) {
                    dlink::TargetPos targetPos;
                    memcpy(&targetPos, payload, sizeof targetPos);
                    // отримані дані таргету
                }
                if (type == dlink::PKT_AMMO) {
                    dlink::AmmoCfg ammoCfg;
                    memcpy(&ammoCfg, payload, sizeof ammoCfg);
                    // отримані дані снаряду
                }
            }
        }
    }
}