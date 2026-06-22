#pragma once

#include <memory>
#include <mutex>

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
    void reset();

private:
    const DroneMotionProfile& motion_profile_;
    std::shared_ptr<DroneConfig> config_ = nullptr;
    std::unique_ptr<IDroneStateMachine> state_;
    DroneState drone_;
    std::unique_ptr<DropPoint> current_destination_ = nullptr;

    DroneTelemetry drone_telemetry_;
    std::mutex telemetry_mutex_;

    void updateTelemetry(DroneTelemetry telemetry);

    float angleToTarget(const Coord& target_position);
    void updatePosition();

    void start() override;
    void run() override;
};
