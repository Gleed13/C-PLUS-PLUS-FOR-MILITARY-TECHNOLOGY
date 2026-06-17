#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "models/Coord.hpp"

class ITargetProvider {
public:
    virtual std::size_t getTargetCount() const = 0;
    virtual std::optional<Coord> getPosition(std::size_t target_index, std::vector<float> params) const = 0;
    virtual ~ITargetProvider() = default;
};
