#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <utility>

#include "features/Logging.hpp"
#include "strategies/JsonTargetProvider.hpp"

std::size_t JsonTargetProvider::getTargetCount() const
{
    return tracks_.size();
}

std::optional<Coord> JsonTargetProvider::getPosition(std::size_t target_index, float time, float sample_interval) const
{
    if (tracks_.empty()) {
        ERROR("Targets are not loaded");
        return std::nullopt;
    }

    if (target_index >= tracks_.size()) {
        ERROR("Target index out of range");
        return std::nullopt;
    }

    if (!std::isfinite(time) || time < 0.0F) {
        ERROR("Target query time must be finite and non-negative");
        return std::nullopt;
    }

    if (!std::isfinite(sample_interval) || sample_interval <= 0.0F) {
        ERROR("Target sample interval must be finite and positive");
        return std::nullopt;
    }

    const auto& positions = tracks_[target_index].positions;
    if (positions.empty()) {
        ERROR("Target track has no positions");
        return std::nullopt;
    }

    const float sample_position = std::fmod(time / sample_interval, static_cast<float>(positions.size()));
    const auto current_index = static_cast<std::size_t>(std::floor(sample_position));
    const auto next_index = (current_index + 1) % positions.size();
    const float fraction = sample_position - static_cast<float>(current_index);

    return positions[current_index] + (positions[next_index] - positions[current_index]) * fraction;
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
    if (!file.is_open()) {
        ERROR(config_path_ << " file error");
        return false;
    }

    json json_data;
    try {
        file >> json_data;
    }
    catch (const json::parse_error& e) {
        ERROR("JSON parse error: " << e.what());
        return false;
    }

    if (!validateTargetsJson(json_data)) {
        return false;
    }

    tracks_.clear();
    const auto& targets = json_data.at("targets");
    tracks_.reserve(targets.size());

    for (const auto& target : targets) {
        TargetTrack track;
        const auto& positions = target.at("positions");
        track.positions.reserve(positions.size());

        for (const auto& position : positions) {
            track.positions.push_back(Coord{position.at("x").get<float>(), position.at("y").get<float>()});
        }

        tracks_.push_back(std::move(track));
    }

    LOG("JSON target provider loaded " << tracks_.size() << " moving targets");

    return true;
}

bool JsonTargetProvider::validateCoordJson(const json& item, std::size_t target_index, std::size_t position_index) const
{
    if (!item.is_object()) {
        ERROR("Position " << position_index << " for target " << target_index << " must be an object");
        return false;
    }

    if (!item.contains("x")) {
        ERROR("Position " << position_index << " for target " << target_index << " is missing field 'x'");
        return false;
    }

    if (!item.contains("y")) {
        ERROR("Position " << position_index << " for target " << target_index << " is missing field 'y'");
        return false;
    }

    if (!item.at("x").is_number()) {
        ERROR("Field 'x' at position " << position_index << " for target " << target_index << " must be a number");
        return false;
    }

    if (!item.at("y").is_number()) {
        ERROR("Field 'y' at position " << position_index << " for target " << target_index << " must be a number");
        return false;
    }

    return true;
}

bool JsonTargetProvider::validateTargetsJson(const json& json_data) const
{
    if (!json_data.is_object()) {
        ERROR("JSON root must be an object");
        return false;
    }

    if (!json_data.contains("targetCount") || !json_data.at("targetCount").is_number_unsigned()) {
        ERROR("Field 'targetCount' must be an unsigned integer");
        return false;
    }

    if (!json_data.contains("timeSteps") || !json_data.at("timeSteps").is_number_unsigned()) {
        ERROR("Field 'timeSteps' must be an unsigned integer");
        return false;
    }

    if (!json_data.contains("targets") || !json_data.at("targets").is_array()) {
        ERROR("Field 'targets' must be an array");
        return false;
    }

    const auto target_count = json_data.at("targetCount").get<std::size_t>();
    const auto time_steps = json_data.at("timeSteps").get<std::size_t>();
    const auto& targets = json_data.at("targets");

    if (target_count > kMaxTargets) {
        ERROR("Too many targets. Max allowed: " << kMaxTargets);
        return false;
    }

    if (target_count != targets.size()) {
        ERROR("Field 'targetCount' does not match the targets array size");
        return false;
    }

    if (time_steps == 0) {
        ERROR("Field 'timeSteps' must be greater than zero");
        return false;
    }

    for (std::size_t target_index = 0; target_index < targets.size(); ++target_index) {
        const auto& target = targets.at(target_index);
        if (!target.is_object() || !target.contains("positions") || !target.at("positions").is_array()) {
            ERROR("Target " << target_index << " must contain a positions array");
            return false;
        }

        const auto& positions = target.at("positions");
        if (positions.size() != time_steps) {
            ERROR("Target " << target_index << " position count does not match timeSteps");
            return false;
        }

        for (std::size_t position_index = 0; position_index < positions.size(); ++position_index) {
            if (!validateCoordJson(positions.at(position_index), target_index, position_index)) {
                return false;
            }
        }
    }

    return true;
}
