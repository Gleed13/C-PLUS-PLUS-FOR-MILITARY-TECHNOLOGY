#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>

#include "interfaces/ITargetProvider.hpp"
#include "models/Coord.hpp"

using json = nlohmann::json;

class JsonTargetProvider : public ITargetProvider {
public:
    JsonTargetProvider(const char* config_path);
    int getTargetCount() override;
    Coord* getTarget(int index) override;
    ~JsonTargetProvider() override;
private:
    static constexpr std::size_t kMaxTargets = 32;

    const char* config_path_;
    Coord* items_ = nullptr;
    std::size_t count_ = 0;

    bool loadConfig();
    auto validateCoordJson(const json& item, std::size_t index) -> bool;
    auto validateTargetsJson(const json& json_data) -> bool;
};