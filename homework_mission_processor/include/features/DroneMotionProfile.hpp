#pragma once

#include "models/DroneConfig.hpp"

class DroneMotionProfile final
{
public:
    bool init(const DroneConfig& config);

    float acceleration() const;
    float remainingAccelerationPath(float speed) const;
    float remainingDecelerationPath(float speed) const;

private:
    float attack_speed_ = 0.0F;
    float acceleration_path_ = 0.0F;
    float acceleration_ = 0.0F;
    bool initialized_ = false;
};
