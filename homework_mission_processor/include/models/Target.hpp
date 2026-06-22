#pragma once

#include <chrono>
#include "models/Coord.hpp"

struct Target {
    Coord pos;      // поточна позиція цілі
    Coord velocity; // поточна швидкість цілі
    std::chrono::steady_clock::time_point lastUpdatedTimePoint = std::chrono::steady_clock::now();
};