#include <fstream>
#include <iostream>
#include <string>

#include "models/Ammo.hpp"
#include "strategies/FileConfigLoader.hpp"

bool FileConfigLoader::tryLoadConfig(const std::string filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cout << "Error: File error" << std::endl;
        return false;
    }

    clearConfig();

    config_ = new DroneConfig();

    file >> config_->startPos.x
         >> config_->startPos.y
         >> config_->altitude
         >> config_->attackSpeed
         >> config_->accelPath
         >> config_->ammoName;

    if (!file)
    {
        std::cout << "Error: Invalid input format" << std::endl;
        clearConfig();
        return false;
    }

    if (!tryLoadAmmoParams())
    {
        clearConfig();
        return false;
    }

    return true;
}

DroneConfig* FileConfigLoader::getConfig() const
{
    return config_;
}

Ammo* FileConfigLoader::getAmmoParams() const
{
    return ammo_params_;
}

FileConfigLoader::~FileConfigLoader()
{
    clearConfig();
    clearAmmoParams();
}

void FileConfigLoader::clearConfig()
{
    delete config_;
    config_ = nullptr;
}

void FileConfigLoader::clearAmmoParams()
{
    delete ammo_params_;
    ammo_params_ = nullptr;
}

bool FileConfigLoader::tryLoadAmmoParams()
{
    auto it = ammoTable.find(config_->ammoName);
    if (it == ammoTable.end())
    {
        std::cout << "Error: Unknown ammo type: " << config_->ammoName << std::endl;
        return false;
    }

    clearAmmoParams();
    ammo_params_ = new Ammo();
    *ammo_params_ = it->second;

    return true;
}