#pragma once

#include <string>
#include <nlohmann/json.hpp>

#include "models/Ammo.hpp"
#include "interfaces/IConfigLoader.hpp"

class JsonConfigLoader : public IConfigLoader {
public:
    bool tryLoadConfig(const std::string filename) override;
    DroneConfig* getConfig() const override;
    Ammo* getAmmoParams() const override;
    ~JsonConfigLoader() override;
private:
    std::unique_ptr<DroneConfig> config_;
    std::unique_ptr<Ammo> ammo_params_;

    void clearConfig();
    void clearAmmoParams();
    bool tryLoadAmmoParams();
};