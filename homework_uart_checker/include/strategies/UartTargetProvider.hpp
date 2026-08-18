#pragma once

#include <cstddef>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "features/UartBridge.hpp"
#include "interfaces/ITargetMotionProvider.hpp"
#include "models/Coord.hpp"
#include "models/DroneConfig.hpp"

using json = nlohmann::json;

class UartTargetProvider : public ITargetMotionProvider {
public:
    ~UartTargetProvider() override;
    UartTargetProvider(std::shared_ptr<UartBridge> uart_bridge);

    bool init(std::shared_ptr<DroneConfig> config) override;
    std::size_t getTargetCount() const override;
    std::optional<Coord> getPosition(std::size_t target_index, std::vector<float> params) const override;
    std::optional<Target> getTarget(std::size_t target_index) const override;

private:
    std::shared_ptr<UartBridge> uart_bridge_;

    std::chrono::steady_clock::time_point start_time_;
    std::vector<std::optional<Target>> targets_;
    mutable std::mutex targets_mutex_;

    void onTargetPositionReceived(const dlink::TargetPos&);
};
