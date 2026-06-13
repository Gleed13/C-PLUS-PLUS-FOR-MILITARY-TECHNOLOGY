#include <algorithm>
#include <cmath>
#include <memory>

#include "features/DroneMovementStates.hpp"

namespace {

template <typename State>
std::unique_ptr<IDroneState> transitionTo(DroneMovementContext& context, DroneStatus next_status)
{
    if (context.drone.status == next_status) {
        return nullptr;
    }

    context.drone.status = next_status;
    return std::make_unique<State>();
}

float distance(const Coord& first, const Coord& second)
{
    return std::hypot(second.x - first.x, second.y - first.y);
}

std::unique_ptr<IDroneState> accelerate(DroneMovementContext& context)
{
    context.drone.speed += context.motionProfile.acceleration() * context.config.simTimeStep;
    if (context.drone.speed >= context.config.attackSpeed) {
        context.drone.speed = context.config.attackSpeed;
        return transitionTo<StateMoving>(context, DroneStatus::Moving);
    }

    return transitionTo<StateAccelerating>(context, DroneStatus::Accelerating);
}

std::unique_ptr<IDroneState> decelerate(DroneMovementContext& context)
{
    context.drone.speed -= context.motionProfile.acceleration() * context.config.simTimeStep;
    if (context.drone.speed <= 0.0F) {
        context.drone.speed = 0.0F;
        return transitionTo<StateStopped>(context, DroneStatus::Stopped);
    }

    return transitionTo<StateDecelerating>(context, DroneStatus::Decelerating);
}

std::unique_ptr<IDroneState> turnOrAccelerate(DroneMovementContext& context)
{
    if (std::abs(context.targetAngle) > context.config.turnThreshold / 2.0F) {
        const float direction_sign = context.targetAngle > 0.0F ? 1.0F : -1.0F;
        context.drone.direction +=
            direction_sign * std::min(context.config.angularSpeed * context.config.simTimeStep, std::abs(context.targetAngle));
        return transitionTo<StateTurning>(context, DroneStatus::Turning);
    }

    return accelerate(context);
}

std::unique_ptr<IDroneState> accelerateOrDecelerate(DroneMovementContext& context)
{
    if (std::abs(context.targetAngle) > context.config.turnThreshold) {
        return decelerate(context);
    }

    if (context.destination.intermPoint.has_value()) {
        const float distance_to_intermediate = distance(context.drone.position, context.targetPosition);
        if (distance_to_intermediate <= context.motionProfile.remainingDecelerationPath(context.drone.speed)) {
            return decelerate(context);
        }
    }

    return accelerate(context);
}

}    // namespace

std::unique_ptr<IDroneState> StateStopped::execute(DroneMovementContext& context) const
{
    return turnOrAccelerate(context);
}

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneMovementContext& context) const
{
    return accelerateOrDecelerate(context);
}

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneMovementContext& context) const
{
    return accelerateOrDecelerate(context);
}

std::unique_ptr<IDroneState> StateTurning::execute(DroneMovementContext& context) const
{
    return turnOrAccelerate(context);
}

std::unique_ptr<IDroneState> StateMoving::execute(DroneMovementContext& context) const
{
    if (std::abs(context.targetAngle) > context.config.turnThreshold) {
        return decelerate(context);
    }

    if (context.destination.intermPoint.has_value()) {
        const float distance_to_intermediate = distance(context.drone.position, context.targetPosition);
        if (distance_to_intermediate <= context.config.accelPath) {
            return decelerate(context);
        }
    }

    context.drone.direction += context.targetAngle;
    return nullptr;
}
