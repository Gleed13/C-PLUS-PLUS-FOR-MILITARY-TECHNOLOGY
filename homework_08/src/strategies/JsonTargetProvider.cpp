#include <fstream>
#include <iostream>
#include <string>

#include "strategies/JsonTargetProvider.hpp"

int JsonTargetProvider::getTargetCount()
{
    return static_cast<int>(items_.size());
}

Coord* JsonTargetProvider::getTarget(int index)
{
    if (items_.empty()) {
        std::cerr << "Error: targets are not loaded\n";
        return nullptr;
    }

    const auto target_index = static_cast<std::size_t>(index);

    if (index < 0 || target_index >= items_.size()) {
        std::cerr << "Error: target index out of range\n";
        return nullptr;
    }

    return &items_[target_index];
}

JsonTargetProvider::JsonTargetProvider(const std::string config_path)
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

    items_.clear();
    if (json_data.size() == 0) {
        return true;
    }

    items_.reserve(json_data.size());

    for (const auto& item : json_data) {
        items_.push_back(Coord{
            item.at("x").get<float>(),
            item.at("y").get<float>()
        });
    }

    std::cout << "Info: json target provider loaded" << std::endl;

    return true;
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

    // simple for loop is used to provide better error messages with target index
    for (std::size_t i = 0; i < json_data.size(); ++i) {
        if (!validateCoordJson(json_data.at(i), i)) {
            return false;
        }
    }

    return true;
}