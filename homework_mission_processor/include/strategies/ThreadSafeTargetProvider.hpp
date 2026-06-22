#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "abstractions/BackgroundService.hpp"
#include "abstractions/IResettable.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "models/Coord.hpp"
#include "models/Target.hpp"
#include "models/TargetTrack.hpp"

using json = nlohmann::json;

class ThreadSafeTargetProvider : public BackgroundService, public ITargetProvider, public IResettable {
public:
    ~ThreadSafeTargetProvider() override;
    ThreadSafeTargetProvider(const std::string config_path, const float time_interval_seconds, const float time_scale = 1.0F);

    std::size_t getTargetCount() const override;
    std::optional<Coord> getPosition(std::size_t target_index, std::vector<float> params) const override;
    std::optional<Target> getTarget(std::size_t target_index) const;

    void reset() override;

private:
    static constexpr std::size_t kMaxTargets = 32;

    const std::string config_path_;
    const float time_interval_seconds_;
    const float time_scale_;

    std::vector<TargetTrack> tracks_;
    std::vector<Target> targets_;
    mutable std::mutex targets_mutex_;

    bool validateTargetsLoadedAndIndex(std::size_t target_index) const;
    bool loadConfig();
    bool validateCoordJson(const json& item, std::size_t target_index, std::size_t position_index) const;
    bool validateTargetsJson(const json& json_data) const;

    void updateTargets();
    void run() override;
};
