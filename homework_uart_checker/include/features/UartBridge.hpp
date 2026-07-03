#pragma once

#include <drone_link.h>

#include <functional>
#include <mutex>
#include <optional>
#include <vector>

#include "abstractions/BackgroundService.hpp"

class UartBridge final : public BackgroundService {
public:
    UartBridge(const char* device_name);

    std::optional<dlink::Telemetry> getTelemetry();
    std::vector<std::optional<dlink::TargetPos>> getTargetPositions();
    std::optional<dlink::AmmoCfg> getAmmoConfig();
    std::optional<dlink::Result> getResult();

    void setTargetPacketHandler(std::function<void(const dlink::TargetPos&)> handler);
    void clearTargetPacketHandler();

    void sendControl(const float accel, const float turn_rate);

    void start() override;

private:
    const char* device_name_;
    int file_descriptor_;

    bool config_inited_ = false;
    std::optional<dlink::Telemetry> telemetry_;
    std::vector<std::optional<dlink::TargetPos>> target_positions_; // we use a vector of optional values so it can be inited with nullopt on resize
    std::optional<dlink::AmmoCfg> ammo_config_;
    std::optional<dlink::Result> result_;

    std::mutex telemetry_mutex_;
    std::mutex target_positions_mutex_;
    std::mutex ammo_config_mutex_;
    std::mutex result_mutex_;

    std::function<void(const dlink::TargetPos&)> target_packet_handler_;
    std::mutex target_packet_handler_mutex_;

    void initializeNumberfTargets(uint8_t number_of_targets);
    void updateTelemetry(dlink::Telemetry telemetry);
    void updateTargetPosition(dlink::TargetPos target_position);
    void setAmmoConfig(dlink::AmmoCfg ammo_config);
    void setResult(dlink::Result result);

    void notifyTargetPacketReceived(const dlink::TargetPos& target_position);

    void handleCompletedPacket(uint8_t type, uint8_t payload[260]);

    void run() override;
};