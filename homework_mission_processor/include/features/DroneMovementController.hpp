#pragma once

#include "features/DroneMotionProfile.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/DropPoint.hpp"

class DroneMovementController final
{
public:
    explicit DroneMovementController(
        const DroneMotionProfile& motion_profile);

    bool init(const DroneConfig& config);

    bool update(
        DroneState& drone,
        const DropPoint& destination) const;

private:
    const DroneMotionProfile& motion_profile_;
    const DroneConfig* config_ = nullptr;

    static float angleToTarget(
        const DroneState& drone,
        const Coord& target_position);
    static float distance(
        const Coord& first,
        const Coord& second);

    static void accelerate(
        DroneState& drone,
        const DroneConfig& config,
        float acceleration);
    static void decelerate(
        DroneState& drone,
        const DroneConfig& config,
        float acceleration);
    static void updatePosition(
        DroneState& drone,
        const DroneConfig& config);
};
