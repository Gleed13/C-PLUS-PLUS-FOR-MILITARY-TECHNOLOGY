#include <algorithm>
#include <cmath>

#include "utils/UartUtils.hpp"

using namespace UartUtils;

float UartUtils::distance(const Coord& first, const Coord& second)
{
    return std::hypot(second.x - first.x, second.y - first.y);
}

float UartUtils::calculateTurnRate(const DroneMovementContext& context, const float delta_time)
{
    const float max_angle_change =
        context.config.angularSpeed * delta_time;

    if (max_angle_change <= 0.0F) {
        return 0.0F;
    }

    return std::clamp(
        context.targetAngle / max_angle_change,
        -1.0F,
        1.0F
    );
}

dlink::Control UartUtils::getControlFromContext(const DroneMovementContext& context, const float delta_time)
{
    switch (context.drone.status) {
        case DroneStatus::Stopped:
            return UartUtils::turnOrAccelerate(context, delta_time);
        case DroneStatus::Accelerating:
            return UartUtils::accelerateOrDecelerate(context, delta_time);
        case DroneStatus::Decelerating:
            return UartUtils::accelerateOrDecelerate(context, delta_time);
        case DroneStatus::Turning:
            return UartUtils::turnOrAccelerate(context, delta_time);
        case DroneStatus::Moving:
            return UartUtils::turnOrDecelerate(context, delta_time);
        default:
            throw std::runtime_error("Unknown drone status");
    }
}

dlink::Control UartUtils::accelerate(const DroneMovementContext& context, const float delta_time)
{
    const float max_speed_change = context.motionProfile.acceleration() * delta_time;
    if (max_speed_change <= 0.0F) {
        return dlink::Control{
            .accel = 0.0F,
            .turnRate = 0.0F
        };
    }
    const float needed_speed_change = context.config.attackSpeed - context.drone.speed;

    const float accel = std::clamp(
        needed_speed_change / max_speed_change,
        0.0F,
        1.0F
    );

    return dlink::Control{
        .accel = accel,
        .turnRate = 0.0F
    };
}

dlink::Control UartUtils::decelerate(const DroneMovementContext& context, const float delta_time)
{
    const float max_speed_change = context.motionProfile.acceleration() * delta_time;
    if (max_speed_change <= 0.0F) {
        return dlink::Control{
            .accel = 0.0F,
            .turnRate = 0.0F
        };
    }
    const float needed_speed_change = 0.0F - context.drone.speed;

    const float accel = std::clamp(
        needed_speed_change / max_speed_change,
        -1.0F,
        0.0F
    );

    return dlink::Control{
        .accel = accel,
        .turnRate = 0.0F
    };
}

dlink::Control UartUtils::turnOrAccelerate(const DroneMovementContext& context, const float delta_time)
{
    if (std::abs(context.targetAngle) > context.config.turnThreshold / 2.0F) {
        const float turn_rate = calculateTurnRate(context, delta_time);

        return dlink::Control{
            .accel = 0.0F,
            .turnRate = turn_rate
        };
    }

    return UartUtils::accelerate(context, delta_time);
}

dlink::Control UartUtils::turnOrDecelerate(const DroneMovementContext& context, const float delta_time)
{
    if (std::abs(context.targetAngle) > context.config.turnThreshold) {
        return UartUtils::decelerate(context, delta_time);
    }

    if (context.destination.intermPoint.has_value()) {
        const float distance_to_intermediate = distance(context.drone.position, context.targetPosition);
        if (distance_to_intermediate <= context.config.accelPath) {
            return UartUtils::decelerate(context, delta_time);
        }
    }

    const float turn_rate = calculateTurnRate(context, delta_time);

    return dlink::Control{
        .accel = 0.0F,
        .turnRate = turn_rate
    };
}

dlink::Control UartUtils::accelerateOrDecelerate(const DroneMovementContext& context, const float delta_time)
{
    if (std::abs(context.targetAngle) > context.config.turnThreshold) {
        return UartUtils::decelerate(context, delta_time);
    }

    if (context.destination.intermPoint.has_value()) {
        const float distance_to_intermediate = distance(context.drone.position, context.targetPosition);
        if (distance_to_intermediate <= context.motionProfile.remainingDecelerationPath(context.drone.speed)) {
            return UartUtils::decelerate(context, delta_time);
        }
    }

    return UartUtils::accelerate(context, delta_time);
}