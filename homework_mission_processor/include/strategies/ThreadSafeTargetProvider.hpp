#pragma once

#include <condition_variable>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "interfaces/ITargetProvider.hpp"
#include "models/Coord.hpp"
#include "models/Target.hpp"
#include "models/TargetTrack.hpp"

using json = nlohmann::json;

class ThreadSafeTargetProvider : public ITargetProvider {
public:
    ThreadSafeTargetProvider(const std::string config_path);
    std::size_t getTargetCount() const override;
    std::optional<Coord> getPosition(std::size_t target_index, std::vector<float> params) const override;
    std::optional<Target> getTarget(std::size_t target_index) const;
    bool simulateTargetsMovement(const float time_interval_seconds);
    bool requestSimulationStop();

  private:
    static constexpr std::size_t kMaxTargets = 32;

    const std::string config_path_;
    std::vector<TargetTrack> tracks_;
    std::vector<Target> targets_;
    mutable std::mutex targets_mutex_;

    bool simulation_stop_requested_ = false;
    std::condition_variable simulation_stop_cv_;
    mutable std::mutex simulation_stop_mutex_;

    bool validateTargetsLoadedAndIndex(std::size_t target_index) const;
    bool loadConfig();
    bool validateCoordJson(const json& item, std::size_t target_index, std::size_t position_index) const;
    bool validateTargetsJson(const json& json_data) const;

    void updateTargets(const float time_interval_seconds);
    bool waitForStopRequested(std::chrono::milliseconds interval);
};
