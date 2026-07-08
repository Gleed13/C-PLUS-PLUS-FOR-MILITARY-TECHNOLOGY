#include <fstream>
#include <string>

#include "features/Logging.hpp"
#include "strategies/JsonConfigLoader.hpp"

bool JsonConfigLoader::tryLoadConfig(const std::string filename)
{
    std::ifstream file(filename);

    if (!file.is_open()) {
        ERROR("File error");
        return false;
    }

    nlohmann::json j;
    file >> j;

    config_ = std::make_unique<DroneConfig>();

    config_->startPos.x = j["drone"]["position"]["x"];
    config_->startPos.y = j["drone"]["position"]["y"];
    config_->altitude = j["drone"]["altitude"];
    config_->initialDir = j["drone"]["initialDirection"];
    config_->attackSpeed = j["drone"]["attackSpeed"];
    config_->accelPath = j["drone"]["accelerationPath"];
    config_->angularSpeed = j["drone"]["angularSpeed"];
    config_->turnThreshold = j["drone"]["turnThreshold"];
    config_->ammoName = j["ammo"];
    config_->simTimeStep = j["simulation"]["timeStep"];
    config_->physicsTimeStep = j["simulation"].value("physicsTimeStep", config_->simTimeStep);
    config_->hitRadius = j["simulation"]["hitRadius"];
    config_->timeScale = j["simulation"].value("timeScale", 1.0F);
    config_->arrayTimeStep = j["targetArrayTimeStep"];

    if (!file) {
        ERROR("Invalid input format");
        config_.reset();
        return false;
    }

    if (!tryLoadAmmoParams()) {
        config_.reset();
        return false;
    }

    LOG("Config loaded");

    return true;
}

std::shared_ptr<DroneConfig> JsonConfigLoader::getConfig() const
{
    return config_;
}

Ammo* JsonConfigLoader::getAmmoParams() const
{
    return ammo_params_.get();
}

bool JsonConfigLoader::tryLoadAmmoParams()
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
