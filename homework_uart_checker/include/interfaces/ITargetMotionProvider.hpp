#pragma once

#include <cstddef>
#include <optional>

#include "interfaces/ITargetProvider.hpp"
#include "models/Target.hpp"

class ITargetMotionProvider : public ITargetProvider {
public:
    virtual std::optional<Target> getTarget(std::size_t target_index) const = 0;
    virtual ~ITargetMotionProvider() = default;
};
