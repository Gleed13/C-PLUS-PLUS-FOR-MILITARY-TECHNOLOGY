#include <cmath>

#include "features/DroneMotionProfile.hpp"

bool DroneMotionProfile::init(const DroneConfig& config)
{
    if (config.attackSpeed <= 0.0F || config.accelPath <= 0.0F) {
        initialized_ = false;
        return false;
    }

    const float acceleration = config.attackSpeed * config.attackSpeed / (2.0F * config.accelPath);
    if (!std::isfinite(acceleration) || acceleration <= 0.0F) {
        initialized_ = false;
        return false;
    }

    attack_speed_ = config.attackSpeed;
    acceleration_path_ = config.accelPath;
    acceleration_ = acceleration;
    initialized_ = true;
    return true;
}

float DroneMotionProfile::acceleration() const
{
    return initialized_ ? acceleration_ : 0.0F;
}

float DroneMotionProfile::remainingAccelerationPath(float speed) const
{
    if (!initialized_ || speed >= attack_speed_) {
        return 0.0F;
    }

    const float speed_ratio = speed / attack_speed_;
    return acceleration_path_ - acceleration_path_ * speed_ratio * speed_ratio;
}

float DroneMotionProfile::remainingDecelerationPath(float speed) const
{
    if (!initialized_ || speed <= 0.0F) {
        return 0.0F;
    }

    const float speed_ratio = speed / attack_speed_;
    return acceleration_path_ * speed_ratio * speed_ratio;
}
