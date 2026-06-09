#pragma once

#include <cstddef>
#include <optional>

#include "models/Coord.hpp"

class ITargetProvider {
public:
    virtual std::size_t getTargetCount() const = 0;
    virtual std::optional<Coord> getPosition(
        std::size_t target_index,
        float time,
        float sample_interval) const = 0;
    virtual ~ITargetProvider() = default;
};
