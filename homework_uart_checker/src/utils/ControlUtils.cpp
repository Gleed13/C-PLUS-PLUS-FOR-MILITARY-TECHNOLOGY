#include <cmath>

#include "utils/ControlUtils.hpp"

using namespace ControlUtils;

float ControlUtils::angleToTarget(const Coord& target_position, const DroneState& drone_)
{
    const Coord to_target = target_position - drone_.position;
    const float target_length = std::hypot(to_target.x, to_target.y);
    if (target_length == 0.0F) {
        return 0.0F;
    }

    const Coord drone_direction{std::cos(drone_.direction), std::sin(drone_.direction)};
    const Coord normalized_target{to_target.x / target_length, to_target.y / target_length};

    const float cross = drone_direction.x * normalized_target.y - drone_direction.y * normalized_target.x;
    const float dot = drone_direction.x * normalized_target.x + drone_direction.y * normalized_target.y;

    return std::atan2(cross, dot);
}