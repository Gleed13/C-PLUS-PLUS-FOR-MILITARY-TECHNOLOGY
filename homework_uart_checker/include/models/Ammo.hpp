#pragma once

#include <map>
#include <string>

struct Ammo {
    std::string ammoType;
    float mass;
    float drag;
    float lift;
};

static const std::map<std::string, Ammo> ammoTable = {{"VOG-17", {"VOG-17", 0.35F, 0.004F, 0.000F}},
                                                      {"M67", {"M67", 0.60F, 0.005F, 0.000F}},
                                                      {"RKG-3", {"RKG-3", 1.20F, 0.007F, 0.000F}},
                                                      {"GLIDING-VOG", {"GLIDING-VOG", 0.45F, 0.005F, 0.005F}},
                                                      {"GLIDING-RKG", {"GLIDING-RKG", 1.40F, 0.007F, 0.005F}}};
