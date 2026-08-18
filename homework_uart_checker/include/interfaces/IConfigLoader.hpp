#pragma once

#include <memory>
#include <string>

#include "models/DroneConfig.hpp"
#include "models/Ammo.hpp"

class IConfigLoader {
public:
    virtual bool tryLoadConfig(const std::string filename) = 0;
    virtual std::shared_ptr<DroneConfig> getConfig() const = 0;
    virtual Ammo* getAmmoParams() const = 0;
    virtual ~IConfigLoader() = default;
};