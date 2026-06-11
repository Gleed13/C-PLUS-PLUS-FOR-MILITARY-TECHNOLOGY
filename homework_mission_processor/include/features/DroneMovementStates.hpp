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
    const DropPoint& destination;
    Coord targetPosition;
    float targetAngle;
};

class IDroneState {
  public:
    virtual ~IDroneState() = default;

    virtual std::unique_ptr<IDroneState> execute(DroneMovementContext& context) const = 0;
};

class StateStopped final : public IDroneState {
  public:
    std::unique_ptr<IDroneState> execute(DroneMovementContext& context) const override;
};

class StateAccelerating final : public IDroneState {
  public:
    std::unique_ptr<IDroneState> execute(DroneMovementContext& context) const override;
};

class StateDecelerating final : public IDroneState {
  public:
    std::unique_ptr<IDroneState> execute(DroneMovementContext& context) const override;
};

class StateTurning final : public IDroneState {
  public:
    std::unique_ptr<IDroneState> execute(DroneMovementContext& context) const override;
};

class StateMoving final : public IDroneState {
  public:
    std::unique_ptr<IDroneState> execute(DroneMovementContext& context) const override;
};
