#include <cstring>
#include <iostream>

#include "strategies/FileConfigLoader.hpp"

bool FileConfigLoader::tryLoadConfig(const char* filename)
{
    FILE* f = fopen(filename, "r");

    if (!f)
    {
        std::cout << "Error: File error" << std::endl;
        return false;
    }

    // delete config data if already loaded
    if (config_ != nullptr) {
        delete config_;
        config_ = nullptr;
    }

    config_ = new DroneConfig();

    int scanned = fscanf(f, "%f %f %f %f %f %49s",
        &config_->startPos.x, &config_->startPos.y, &config_->altitude,
        &config_->attackSpeed, &config_->accelPath,
        config_->ammoName);

    fclose(f);

    if (scanned != 6)
    {
        std::cout << "Error: Invalid input format" << std::endl;
        delete config_;
        config_ = nullptr;
        return false;
    }

    if (!tryLoadAmmoParams())
    {
        delete config_;
        config_ = nullptr;
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
    delete config_;
    config_ = nullptr;
    delete ammo_params_;
    ammo_params_ = nullptr;
}

bool FileConfigLoader::tryLoadAmmoParams()
{
    ammo_params_ = new Ammo();
    int count = sizeof(ammoTable) / sizeof(ammoTable[0]);

    for (int i = 0; i < count; i++)
    {
        if (strcmp(ammoTable[i].ammoType, config_->ammoName) == 0)
        {
            ammo_params_->ammoType = ammoTable[i].ammoType;
            ammo_params_->mass = ammoTable[i].mass;
            ammo_params_->drag = ammoTable[i].drag;
            ammo_params_->lift = ammoTable[i].lift;
            return true;
        }
    }

    std::cout << "Error: Unknown ammo type: " << config_->ammoName << std::endl;
    delete ammo_params_;
    ammo_params_ = nullptr;

    return false;
}