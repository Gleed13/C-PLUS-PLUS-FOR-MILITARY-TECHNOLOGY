#pragma once

#include <memory>

#include "features/DroneMotionProfile.hpp"
#include "features/DroneMovementStates.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/DropPoint.hpp"

class DroneMovementController final
{
public:
    explicit DroneMovementController(
        const DroneMotionProfile& motion_profile);

    bool init(const DroneConfig& config);
    void reset();

    bool update(
        DroneState& drone,
        const DropPoint& destination);

private:
    const DroneMotionProfile& motion_profile_;
    const DroneConfig* config_ = nullptr;
    std::unique_ptr<IDroneState> state_;

    static float angleToTarget(
        const DroneState& drone,
        const Coord& target_position);
    static void updatePosition(
        DroneState& drone,
        const DroneConfig& config);
};
