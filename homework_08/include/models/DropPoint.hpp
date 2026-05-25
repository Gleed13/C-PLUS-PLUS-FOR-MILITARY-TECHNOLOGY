#pragma once

#include <optional>

#include "Coord.hpp"

struct DropPoint
{
    std::optional<Coord> intermPoint;
    Coord firePoint;
};