#pragma once

#include <memory>

#include "features/DroneMotionProfile.hpp"
#include "models/DroneConfig.hpp"
#include "models/DroneState.hpp"
#include "models/DropPoint.hpp"

struct DroneMovementContext {
    DroneState& drone;
    const DroneConfig& config;
    const DroneMotionProfile& motionProfile;
    const DropPoint destination;
    const Coord targetPosition;
    const float targetAngle;
};

class IDroneStateMachine {
public:
    virtual ~IDroneStateMachine() = default;

    virtual std::unique_ptr<IDroneStateMachine> execute(DroneMovementContext& context) const = 0;
};

class StateStopped final : public IDroneStateMachine {
public:
    std::unique_ptr<IDroneStateMachine> execute(DroneMovementContext& context) const override;
};

class StateAccelerating final : public IDroneStateMachine {
public:
    std::unique_ptr<IDroneStateMachine> execute(DroneMovementContext& context) const override;
};

class StateDecelerating final : public IDroneStateMachine {
public:
    std::unique_ptr<IDroneStateMachine> execute(DroneMovementContext& context) const override;
};

class StateTurning final : public IDroneStateMachine {
public:
    std::unique_ptr<IDroneStateMachine> execute(DroneMovementContext& context) const override;
};

class StateMoving final : public IDroneStateMachine {
public:
    std::unique_ptr<IDroneStateMachine> execute(DroneMovementContext& context) const override;
};
