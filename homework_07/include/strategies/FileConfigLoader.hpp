#pragma once

#include "interfaces/IConfigLoader.hpp"

class FileConfigLoader : public IConfigLoader {
public:
    bool tryLoadConfig(const char* filename) override;
    DroneConfig* getConfig() const override;
    Ammo* getAmmoParams() const override;
    ~FileConfigLoader() override;
private:
    DroneConfig* config_;
    Ammo* ammo_params_;

    bool tryLoadAmmoParams();
};