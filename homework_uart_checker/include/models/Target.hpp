#pragma once

#include "models/Coord.hpp"

struct Target {
    Coord pos;      // поточна позиція цілі
    Coord velocity; // поточна швидкість цілі
    float timeSecSinceStart; // час у секундах від початку симуляції, коли ціль була оновлена
};