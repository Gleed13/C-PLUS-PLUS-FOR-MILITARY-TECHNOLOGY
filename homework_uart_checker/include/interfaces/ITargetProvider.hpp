#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "models/Coord.hpp"
#include "models/DroneConfig.hpp"

class ITargetProvider {
public:
    virtual bool init(std::shared_ptr<DroneConfig> config) = 0;
    virtual std::size_t getTargetCount() const = 0;
    virtual std::optional<Coord> getPosition(std::size_t target_index, std::vector<float> params) const = 0;
    virtual ~ITargetProvider() = default;
};
