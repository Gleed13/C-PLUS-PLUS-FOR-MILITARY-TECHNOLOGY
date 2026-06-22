#pragma once

#include <string>
#include <nlohmann/json.hpp>

#include "models/Ammo.hpp"
#include "interfaces/IConfigLoader.hpp"

class JsonConfigLoader : public IConfigLoader {
public:
    bool tryLoadConfig(const std::string filename) override;
    std::shared_ptr<DroneConfig> getConfig() const override;
    Ammo* getAmmoParams() const override;

private:
    std::shared_ptr<DroneConfig> config_;
    std::unique_ptr<Ammo> ammo_params_;

    bool tryLoadAmmoParams();
};
