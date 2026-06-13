#pragma once

#include <string>

#include "interfaces/IConfigLoader.hpp"

class FileConfigLoader : public IConfigLoader {
public:
    bool tryLoadConfig(const std::string filename) override;
    DroneConfig* getConfig() const override;
    Ammo* getAmmoParams() const override;
    ~FileConfigLoader() override;
private:
    DroneConfig* config_;
    Ammo* ammo_params_;

    void clearConfig();
    void clearAmmoParams();
    bool tryLoadAmmoParams();
};