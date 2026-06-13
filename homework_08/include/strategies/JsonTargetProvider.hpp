#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "interfaces/ITargetProvider.hpp"
#include "models/Coord.hpp"

using json = nlohmann::json;

class JsonTargetProvider : public ITargetProvider {
public:
    JsonTargetProvider(const std::string config_path);
    int getTargetCount() override;
    Coord* getTarget(int index) override;
private:
    static constexpr std::size_t kMaxTargets = 32;

    const std::string config_path_;
    std::vector<Coord> items_;

    bool loadConfig();
    auto validateCoordJson(const json& item, std::size_t index) -> bool;
    auto validateTargetsJson(const json& json_data) -> bool;
};