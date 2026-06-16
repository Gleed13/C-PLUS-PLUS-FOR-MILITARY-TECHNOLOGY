#include <fstream>
#include <string>

#include "features/Logging.hpp"
#include "models/Ammo.hpp"
#include "strategies/FileConfigLoader.hpp"

bool FileConfigLoader::tryLoadConfig(const std::string filename)
{
    std::ifstream file(filename);

    if (!file.is_open()) {
        ERROR("File error");
        return false;
    }

    config_ = std::make_unique<DroneConfig>();

    file >> config_->startPos.x >> config_->startPos.y >> config_->altitude >> config_->initialDir >> config_->attackSpeed >>
        config_->accelPath >> config_->angularSpeed >> config_->turnThreshold >> config_->ammoName >> config_->simTimeStep >>
        config_->hitRadius >> config_->arrayTimeStep;

    if (!file) {
        ERROR("Invalid input format");
        config_.reset();
        return false;
    }

    if (!tryLoadAmmoParams()) {
        config_.reset();
        return false;
    }

    return true;
}

DroneConfig* FileConfigLoader::getConfig() const
{
    return config_.get();
}

Ammo* FileConfigLoader::getAmmoParams() const
{
    return ammo_params_.get();
}

bool FileConfigLoader::tryLoadAmmoParams()
{
    auto it = ammoTable.find(config_->ammoName);
    if (it == ammoTable.end()) {
        ERROR("Unknown ammo type: " << config_->ammoName);
        return false;
    }

    ammo_params_ = std::make_unique<Ammo>();
    *ammo_params_ = it->second;

    return true;
}
