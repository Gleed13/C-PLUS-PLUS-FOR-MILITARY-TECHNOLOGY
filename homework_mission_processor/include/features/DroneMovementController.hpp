#pragma once

#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/DropPoint.hpp"

class DroneMovementController final
{
public:
    bool update(
        DroneState& drone,
        const DropPoint& destination,
        const DroneConfig& config) const;

    float remainingAccelerationPath(
        const DroneState& drone,
        const DroneConfig& config) const;
    float remainingDecelerationPath(
        const DroneState& drone,
        const DroneConfig& config) const;

private:
    static float calculateAcceleration(const DroneConfig& config);
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
