#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "interfaces/ITargetProvider.hpp"
#include "models/Coord.hpp"
#include "models/TargetTrack.hpp"

using json = nlohmann::json;

class JsonTargetProvider : public ITargetProvider {
public:
    JsonTargetProvider(const std::string config_path);
    std::size_t getTargetCount() const override;
    std::optional<Coord> getPosition(std::size_t target_index, float time, float sample_interval) const override;

private:
    static constexpr std::size_t kMaxTargets = 32;

    const std::string config_path_;
    std::vector<TargetTrack> tracks_;

    bool loadConfig();
    bool validateCoordJson(const json& item, std::size_t target_index, std::size_t position_index) const;
    bool validateTargetsJson(const json& json_data) const;
};
