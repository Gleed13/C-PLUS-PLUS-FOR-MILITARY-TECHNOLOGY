#pragma once

#include "models/DroneConfig.hpp"
#include "models/Ammo.hpp"

class IConfigLoader {
public:
    virtual bool tryLoadConfig(const char* filename) = 0;
    virtual DroneConfig* getConfig() const = 0;
    virtual Ammo* getAmmoParams() const = 0;
    virtual ~IConfigLoader() = default;
};