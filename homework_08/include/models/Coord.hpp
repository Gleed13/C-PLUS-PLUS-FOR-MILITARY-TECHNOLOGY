#pragma once

struct Coord
{
    float x;
    float y;

    Coord operator+(const Coord& other) const
    {
        return {x + other.x, y + other.y};
    }
    Coord operator-(const Coord& other) const
    {
        return {x - other.x, y - other.y};
    }
    Coord operator*(float s) const
    {
        return {x * s, y * s};
    }
    Coord operator/(float s) const
    {
        return {x / s, y / s};
    }
    bool operator==(const Coord& other) const
    {
        return x == other.x && y == other.y;
    }
};