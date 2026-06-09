#include <fstream>
#include <string>

#include "features/Logging.hpp"
#include "strategies/JsonTargetProvider.hpp"

int JsonTargetProvider::getTargetCount()
{
    return static_cast<int>(items_.size());
}

Coord* JsonTargetProvider::getTarget(int index)
{
    if (items_.empty()) {
        ERROR("Targets are not loaded");
        return nullptr;
    }

    const auto target_index = static_cast<std::size_t>(index);

    if (index < 0 || target_index >= items_.size()) {
        ERROR("Target index out of range");
        return nullptr;
    }

    return &items_[target_index];
}

JsonTargetProvider::JsonTargetProvider(const std::string config_path)
    : config_path_(config_path)
{
    if (!loadConfig()) {
        ERROR("Failed to load JSON target provider");
    }
}

bool JsonTargetProvider::loadConfig()
{
    std::ifstream file(config_path_);
    if (!file.is_open())
    {
        ERROR(config_path_ << " file error");
        return false;
    }

    json json_data;
    try {
        file >> json_data;
    } catch (const json::parse_error& e) {
        ERROR("JSON parse error: " << e.what());
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

    LOG("JSON target provider loaded");

    return true;
}

auto JsonTargetProvider::validateCoordJson(const json& item, std::size_t index) -> bool {
    if (!item.is_object()) {
        ERROR("Target at index " << index << " must be an object");
        return false;
    }

    if (!item.contains("x")) {
        ERROR("Target at index " << index << " is missing field 'x'");
        return false;
    }

    if (!item.contains("y")) {
        ERROR("Target at index " << index << " is missing field 'y'");
        return false;
    }

    if (!item.at("x").is_number()) {
        ERROR("Field 'x' at index " << index << " must be a number");
        return false;
    }

    if (!item.at("y").is_number()) {
        ERROR("Field 'y' at index " << index << " must be a number");
        return false;
    }

    return true;
}

auto JsonTargetProvider::validateTargetsJson(const json& json_data) -> bool {
    if (!json_data.is_array()) {
        ERROR("JSON root must be an array");
        return false;
    }

    if (json_data.size() > kMaxTargets) {
        ERROR("Too many targets. Max allowed: " << kMaxTargets);
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
