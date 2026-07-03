#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "abstractions/BackgroundService.hpp"
#include "abstractions/IResettable.hpp"
#include "interfaces/ITargetMotionProvider.hpp"
#include "models/Coord.hpp"
#include "models/DroneConfig.hpp"
#include "models/Target.hpp"
#include "models/TargetTrack.hpp"

using json = nlohmann::json;

class ThreadSafeTargetProvider : public BackgroundService, public ITargetMotionProvider, public IResettable {
public:
    ~ThreadSafeTargetProvider() override;
    ThreadSafeTargetProvider(const std::string config_path);

    bool init(std::shared_ptr<DroneConfig> config) override;
    std::size_t getTargetCount() const override;
    std::optional<Coord> getPosition(std::size_t target_index, std::vector<float> params) const override;
    std::optional<Target> getTarget(std::size_t target_index) const override;

    bool isThreadReady() const;
    void reset() override;

private:
    static constexpr std::size_t kMaxTargets = 32;

    const std::string config_path_;
    std::shared_ptr<DroneConfig> config_ = nullptr;

    std::vector<TargetTrack> tracks_;
    std::vector<Target> targets_;
    mutable std::mutex targets_mutex_;

    bool validateTargetsLoadedAndIndex(std::size_t target_index) const;
    bool loadConfig();
    bool validateCoordJson(const json& item, std::size_t target_index, std::size_t position_index) const;
    bool validateTargetsJson(const json& json_data) const;

    void updateTargets(int track_index, float time_sec_since_start);
    void run() override;
};
