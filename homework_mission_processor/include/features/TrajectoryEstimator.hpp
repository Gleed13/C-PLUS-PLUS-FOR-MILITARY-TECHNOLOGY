#pragma once

#include <optional>

#include "features/DroneMovementController.hpp"
#include "models/Coord.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/DropPoint.hpp"

class TrajectoryEstimator final
{
public:
    std::optional<float> estimateArrivalTime(
        const DroneState& drone,
        const Coord& target_position,
        const DropPoint& drop_point,
        const DroneConfig& config) const;

private:
    DroneMovementController movement_controller_;

    static float angleToTarget(
        const Coord& position,
        float direction,
        const Coord& target_position);
    static float distance(
        const Coord& first,
        const Coord& second);
};
