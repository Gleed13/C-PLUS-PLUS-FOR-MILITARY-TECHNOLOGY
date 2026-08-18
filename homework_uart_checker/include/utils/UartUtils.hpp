#pragma once

#include "drone_link.h"
#include "features/DroneMovementStates.hpp"

namespace UartUtils {
    float distance(const Coord& first, const Coord& second);
    float calculateTurnRate(const DroneMovementContext& context, const float delta_time);
    dlink::Control getControlFromContext(const DroneMovementContext& context, const float delta_time);
    dlink::Control accelerate(const DroneMovementContext& context, const float delta_time);
    dlink::Control decelerate(const DroneMovementContext& context, const float delta_time);
    dlink::Control turnOrAccelerate(const DroneMovementContext& context, const float delta_time);
    dlink::Control turnOrDecelerate(const DroneMovementContext& context, const float delta_time);
    dlink::Control accelerateOrDecelerate(const DroneMovementContext& context, const float delta_time);
}