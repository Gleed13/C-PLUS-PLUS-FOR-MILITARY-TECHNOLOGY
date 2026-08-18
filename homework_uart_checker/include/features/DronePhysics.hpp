#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include "abstractions/BackgroundService.hpp"
#include "features/DroneMotionProfile.hpp"
#include "features/DroneMovementStates.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/DropPoint.hpp"

class DronePhysics final : public BackgroundService{
public:
    explicit DronePhysics(const DroneMotionProfile& motion_profile);

    bool init(std::shared_ptr<DroneConfig> config);
    bool updateDestination(const DropPoint* destination);
    DroneTelemetry getTelemetry();
    void start(std::shared_ptr<std::latch> ready_latch, std::shared_ptr<std::latch> start_gate) override;
    void reset();

private:
    const DroneMotionProfile& motion_profile_;
    std::shared_ptr<DroneConfig> config_ = nullptr;
    std::unique_ptr<IDroneStateMachine> state_ = nullptr;
    DroneState drone_;

    std::optional<DropPoint> current_destination_ = std::nullopt;
    std::mutex destination_mutex_;

    DroneTelemetry drone_telemetry_;
    std::mutex telemetry_mutex_;

    void updateTelemetry(DroneTelemetry telemetry);
    void updatePosition();
    std::optional<DroneMovementContext> createMovementContext();

    void run() override;
};
