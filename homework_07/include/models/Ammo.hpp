#pragma once

struct Ammo
{
    const char* ammoType;
    float mass;
    float drag;
    float lift;
};

static constexpr Ammo ammoTable[] =
{
    {"VOG-17",      0.35f, 0.07f, 0.0f},
    {"M67",         0.60f, 0.10f, 0.0f},
    {"RKG-3",       1.20f, 0.10f, 0.0f},
    {"GLIDING-VOG", 0.45f, 0.10f, 1.0f},
    {"GLIDING-RKG", 1.40f, 0.10f, 1.0f}
};