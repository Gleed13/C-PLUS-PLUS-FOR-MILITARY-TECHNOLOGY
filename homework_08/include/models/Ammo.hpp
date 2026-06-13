#pragma once

#include <map>
#include <string>

struct Ammo
{
    std::string ammoType;
    float mass;
    float drag;
    float lift;
};

static const std::map<std::string, Ammo> ammoTable =
{
    {"VOG-17",      { "VOG-17",      0.35f, 0.07f, 0.0f }},
    {"M67",         { "M67",         0.60f, 0.10f, 0.0f }},
    {"RKG-3",       { "RKG-3",       1.20f, 0.10f, 0.0f }},
    {"GLIDING-VOG", { "GLIDING-VOG", 0.45f, 0.10f, 1.0f }},
    {"GLIDING-RKG", { "GLIDING-RKG", 1.40f, 0.10f, 1.0f }}
};