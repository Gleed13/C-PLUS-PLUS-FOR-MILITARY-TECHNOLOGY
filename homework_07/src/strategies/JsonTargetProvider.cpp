#include <fstream>
#include <iostream>

#include "strategies/JsonTargetProvider.hpp"

int JsonTargetProvider::getTargetCount()
{
    return static_cast<int>(count_);
}

Coord* JsonTargetProvider::getTarget(int index)
{
    if (items_ == nullptr) {
        std::cerr << "Error: targets are not loaded\n";
        return nullptr;
    }

    if (index < 0) {
        std::cerr << "Error: target index out of range\n";
        return nullptr;
    }

    const auto target_index = static_cast<std::size_t>(index);

    if (target_index >= count_) {
        std::cerr << "Error: target index out of range\n";
        return nullptr;
    }

    return &items_[target_index];
}

JsonTargetProvider::JsonTargetProvider(const char* config_path)
    : config_path_(config_path)
{
    if (!loadConfig()) {
        std::cerr << "Error: failed to load JSON target provider\n";
    }
}

bool JsonTargetProvider::loadConfig()
{
    std::ifstream file(config_path_);
    if (!file.is_open())
    {
        std::cerr << "Error: " << config_path_ << " file error\n";
        return false;
    }

    json json_data;
    try {
        file >> json_data;
    } catch (const json::parse_error& e) {
        std::cerr << "Error: JSON parse error: " << e.what() << '\n';
        return false;
    }

    if (!validateTargetsJson(json_data)) {
        return false;
    }

    count_ = json_data.size();

    if (count_ == 0) {
        return true;
    }

    items_ = new Coord[count_];

    for (std::size_t i = 0; i < count_; ++i) {
        const auto& item = json_data.at(i);

        items_[i].x = item.at("x").get<float>();
        items_[i].y = item.at("y").get<float>();
    }

    std::cout << "Info: json target provider loaded" << std::endl;

    return true;
}

JsonTargetProvider::~JsonTargetProvider()
{
    delete[] items_;
    items_ = nullptr;
    count_ = 0;
}

auto JsonTargetProvider::validateCoordJson(const json& item, std::size_t index) -> bool {
    if (!item.is_object()) {
        std::cerr << "Error: target at index " << index << " must be an object\n";
        return false;
    }

    if (!item.contains("x")) {
        std::cerr << "Error: target at index " << index << " is missing field 'x'\n";
        return false;
    }

    if (!item.contains("y")) {
        std::cerr << "Error: target at index " << index << " is missing field 'y'\n";
        return false;
    }

    if (!item.at("x").is_number()) {
        std::cerr << "Error: field 'x' at index " << index << " must be a number\n";
        return false;
    }

    if (!item.at("y").is_number()) {
        std::cerr << "Error: field 'y' at index " << index << " must be a number\n";
        return false;
    }

    return true;
}

auto JsonTargetProvider::validateTargetsJson(const json& json_data) -> bool {
    if (!json_data.is_array()) {
        std::cerr << "Error: JSON root must be an array\n";
        return false;
    }

    if (json_data.size() > kMaxTargets) {
        std::cerr << "Error: too many targets. Max allowed: " << kMaxTargets << '\n';
        return false;
    }

    for (std::size_t i = 0; i < json_data.size(); ++i) {
        if (!validateCoordJson(json_data.at(i), i)) {
            return false;
        }
    }

    return true;
}