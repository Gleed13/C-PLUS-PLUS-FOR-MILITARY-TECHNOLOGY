#pragma once

#include <memory>
#include <string>

#include "interfaces/IConfigLoader.hpp"

class FileConfigLoader : public IConfigLoader {
public:
    bool tryLoadConfig(const std::string filename) override;
    DroneConfig* getConfig() const override;
    Ammo* getAmmoParams() const override;

private:
    std::unique_ptr<DroneConfig> config_;
    std::unique_ptr<Ammo> ammo_params_;

    bool tryLoadAmmoParams();
};
