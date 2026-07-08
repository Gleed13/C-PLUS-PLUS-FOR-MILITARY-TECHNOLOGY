#include <cmath>
#include <optional>

#include "features/Logging.hpp"
#include "strategies/UartTargetProvider.hpp"

UartTargetProvider::~UartTargetProvider() {
    if (uart_bridge_) {
        uart_bridge_->clearTargetPacketHandler();
    }
}

UartTargetProvider::UartTargetProvider(std::shared_ptr<UartBridge> uart_bridge) : uart_bridge_(uart_bridge) {
    if (uart_bridge_ == nullptr) {
        ERROR("UartBridge pointer is null");
        return;
    }
}

bool UartTargetProvider::init(std::shared_ptr<DroneConfig> config)
{
    if (config != nullptr) {
        WARNING("Drone configuration is not needed for UartTargetProvider, it will be ignored");
    }

    if (!uart_bridge_) {
        ERROR("UartBridge is not set for UartTargetProvider");
        return false;
    }

    uart_bridge_->setTargetPacketHandler([this](const dlink::TargetPos& target_pos) {
        onTargetPositionReceived(target_pos);
    });
    start_time_ = std::chrono::steady_clock::now();

    return true;
}

std::size_t UartTargetProvider::getTargetCount() const
{
    return uart_bridge_->getTargetPositions().size();
}

std::optional<Coord> UartTargetProvider::getPosition(std::size_t target_index, std::vector<float> params) const
{
    if (!params.empty()) {
        WARNING("Parameters are not needed for UartTargetProvider, they will be ignored");
    }

    auto target_positions = uart_bridge_->getTargetPositions();
    if (target_index >= target_positions.size()) {
        WARNING("Target index " + std::to_string(target_index) + " is out of bounds, max index is " + std::to_string(target_positions.size() - 1));
        return std::nullopt;
    }
    auto target_position = target_positions[target_index];
    if (!target_position.has_value()) {
        WARNING("Target position for index " + std::to_string(target_index) + " is not available");
        return std::nullopt;
    }

    return Coord{ target_position->x, target_position->y };
}

std::optional<Target> UartTargetProvider::getTarget(std::size_t target_index) const {
    std::lock_guard<std::mutex> lock(targets_mutex_);
    if (target_index >= targets_.size()) {
        DEBUG("Target index " + std::to_string(target_index) + " is out of bounds, max index is " + std::to_string(targets_.size() - 1));
        return std::nullopt;
    }
    return targets_[target_index];
}

void UartTargetProvider::onTargetPositionReceived(const dlink::TargetPos& target_position)
{
    auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(targets_mutex_);
    if (target_position.id >= targets_.size()) {
        targets_.resize(target_position.id + 1);
    }
    auto last_target = targets_[target_position.id];
    auto time_sec_since_start = std::chrono::duration<float>(now - start_time_).count();
    if (!last_target.has_value()) {
        targets_[target_position.id] = Target{
            Coord{ target_position.x, target_position.y },
            Coord{0, 0},
            time_sec_since_start
        };
        return;
    }

    float time_delta = time_sec_since_start - last_target->timeSecSinceStart;
    if (time_delta <= 0.0F) {
        WARNING("Received target position with non-positive time delta: " + std::to_string(time_delta) + " seconds");
        return;
    }
    Coord velocity{
        (target_position.x - last_target->pos.x) / time_delta,
        (target_position.y - last_target->pos.y) / time_delta
    };
    targets_[target_position.id] = Target{
        Coord{ target_position.x, target_position.y },
        velocity,
        time_sec_since_start
    };
}