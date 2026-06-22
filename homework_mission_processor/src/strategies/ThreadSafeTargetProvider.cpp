#include <cmath>
#include <fstream>
#include <optional>
#include <utility>

#include "features/Logging.hpp"
#include "models/TargetTrack.hpp"
#include "strategies/ThreadSafeTargetProvider.hpp"

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() {
    stop();
}

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string config_path, const float time_interval_seconds, const float time_scale)
    : config_path_(config_path)
    , time_interval_seconds_(time_interval_seconds)
    , time_scale_(time_scale)
{
    if (!loadConfig()) {
        ERROR("Failed to load JSON target provider");
    }
}

std::size_t ThreadSafeTargetProvider::getTargetCount() const
{
    return tracks_.size();
}

std::optional<Coord> ThreadSafeTargetProvider::getPosition(std::size_t target_index, std::vector<float> params) const
{
    if (!validateTargetsLoadedAndIndex(target_index)) {
        return std::nullopt;
    }

    if (params.size() != 0) {
        ERROR("Parameters are not used in this implementation");
        return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(targets_mutex_);
    return targets_[target_index].pos;
}

std::optional<Target> ThreadSafeTargetProvider::getTarget(std::size_t target_index) const
{
    if (!validateTargetsLoadedAndIndex(target_index)) {
        return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(targets_mutex_);
    return targets_[target_index];
}

void ThreadSafeTargetProvider::reset() {
    stop();
    std::lock_guard<std::mutex> lock(targets_mutex_);
    targets_.clear();
}

bool ThreadSafeTargetProvider::validateTargetsLoadedAndIndex(std::size_t target_index) const
{
    if (tracks_.empty()) {
        ERROR("Targets are not loaded");
        return false;
    }

    if (target_index >= tracks_.size()) {
        ERROR("Target index out of range");
        return false;
    }

    return true;
}

bool ThreadSafeTargetProvider::loadConfig()
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

bool ThreadSafeTargetProvider::validateCoordJson(const json& item, std::size_t target_index, std::size_t position_index) const
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

bool ThreadSafeTargetProvider::validateTargetsJson(const json& json_data) const
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

void ThreadSafeTargetProvider::updateTargets()
{
    std::lock_guard<std::mutex> lock(targets_mutex_);
    auto current_time_point = std::chrono::steady_clock::now();

    if (targets_.empty()) {
        for (const TargetTrack& track : tracks_) {
            if (!track.positions.empty()) {
                targets_.push_back(Target{track.positions.front(), Coord{0.0F, 0.0F}, current_time_point});
            }
        }
        return;
    }

    for (std::size_t i = 0; i < targets_.size(); ++i) {
        const TargetTrack& track = tracks_[i];

        if (!track.positions.empty()) {
            const auto& current_pos = targets_[i].pos;
            const auto& next_pos = track.positions.size() > i + 1 ? track.positions[i + 1] : track.positions.front();
            targets_[i].velocity = (next_pos - current_pos) / time_interval_seconds_;
            targets_[i].pos = next_pos;
            targets_[i].lastUpdatedTimePoint = current_time_point;
        }
    }
}

void ThreadSafeTargetProvider::run()
{
    const auto interval = std::chrono::milliseconds(
        static_cast<int>(time_interval_seconds_ * 1000.0F / time_scale_));

    while (!stop_requested()) {
        auto step_start_time = std::chrono::steady_clock::now();

        updateTargets();

        auto step_end_time = std::chrono::steady_clock::now();
        auto step_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(step_end_time - step_start_time);
        auto remaining_sleep_time = interval - step_elapsed_time;
        if (remaining_sleep_time <= std::chrono::nanoseconds(0)) {
            continue;
        }

        bool stopped = wait_for(remaining_sleep_time);
        if (stopped) {
            break;
        }
    }
}