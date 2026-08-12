#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

#include "antidrone_turret/target_decision_logic.hpp"
#include "antidrone_turret/turret_status.hpp"

namespace {

constexpr auto kQoSHistoryDepth = 10;
constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kGimbalCmdTopic = "/gimbal/cmd";
constexpr auto kServoCmdTopic = "/servo/cmd";
constexpr auto kActuatorTriggerService = "/actuator/trigger";
constexpr auto kTurretStatusTopic = "/turret/status";

antidrone_turret::TargetInput to_input(const antidrone_turret::msg::Target& target)
{
  auto input = antidrone_turret::TargetInput{};
  input.visible = target.visible;
  input.x = target.x;
  input.y = target.y;
  input.distance_m = target.distance_m;
  input.confidence = target.confidence;
  return input;
}

antidrone_turret::msg::TurretStatus to_message(const antidrone_turret::TurretStatus& status)
{
  auto message = antidrone_turret::msg::TurretStatus{};
  message.target_state = static_cast<std::uint8_t>(status.target_state);
  message.action = static_cast<std::uint8_t>(status.action);
  message.trigger_state = static_cast<std::uint8_t>(status.trigger_state);
  message.confidence = status.confidence;
  message.distance_m = status.distance_m;
  return message;
}

antidrone_turret::msg::ServoCommand to_message(const antidrone_turret::ServoCommand& command)
{
  auto message = antidrone_turret::msg::ServoCommand{};
  message.direction = static_cast<std::int8_t>(command.direction);
  message.target_x = command.target_x;
  message.error_x = command.error_x;
  return message;
}

antidrone_turret::msg::GimbalCommand to_message(const antidrone_turret::GimbalCommand& command)
{
  auto message = antidrone_turret::msg::GimbalCommand{};
  message.direction = static_cast<std::int8_t>(command.direction);
  message.target_y = command.target_y;
  message.error_y = command.error_y;
  return message;
}

}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  using TargetTrack = antidrone_turret::msg::Target;
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using GimbalCmd = antidrone_turret::msg::GimbalCommand;
  using ServoCmd = antidrone_turret::msg::ServoCommand;
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    config_.confidence_threshold =
      static_cast<float>(declare_parameter<double>("confidence_threshold", 0.8));
    config_.max_distance_m =
      static_cast<float>(declare_parameter<double>("max_distance_m", 30.0));

    target_track_subscription_ = create_subscription<TargetTrack>(
      kTargetTopic,
      kQoSHistoryDepth,
      [this](const TargetTrack& target_track) {
        on_target_track(target_track);
      });
    actuator_status_subscription_ = create_subscription<ActuatorStatus>(
      kActuatorStatusTopic,
      kQoSHistoryDepth,
      [this](const ActuatorStatus& actuator_status) {
        on_actuator_status(actuator_status);
      });
    gimbal_cmd_publisher_ = create_publisher<GimbalCmd>(kGimbalCmdTopic, kQoSHistoryDepth);
    servo_cmd_publisher_ = create_publisher<ServoCmd>(kServoCmdTopic, kQoSHistoryDepth);
    turret_status_publisher_ = create_publisher<TurretStatus>(kTurretStatusTopic, kQoSHistoryDepth);
    trigger_client_ = create_client<TriggerActuator>(kActuatorTriggerService);

    RCLCPP_INFO(
      get_logger(),
      "turret controller started with confidence_threshold=%.2f max_distance_m=%.1f",
      config_.confidence_threshold,
      config_.max_distance_m);
  }

private:
  void on_target_track(const TargetTrack& target_track)
  {
    const auto target = to_input(target_track);
    const auto status = antidrone_turret::decide(target, config_, actuator_state());

    // Наведення публікується тільки для надійно захопленої цілі.
    if (status.action == antidrone_turret::TurretAction::kTrack) {
      const auto servo_command = antidrone_turret::make_servo_command(target.x);
      const auto gimbal_command = antidrone_turret::make_gimbal_command(target.y);

      servo_cmd_publisher_->publish(to_message(servo_command));
      gimbal_cmd_publisher_->publish(to_message(gimbal_command));
    }

    // Статус публікується для кожного повідомлення цілі, щоб рішення було
    // видно через ros2 topic echo /turret/status.
    turret_status_publisher_->publish(to_message(status));

    RCLCPP_INFO(
      get_logger(),
      "target=%s action=%s trigger=%s distance_m=%.1f confidence=%.2f",
      antidrone_turret::to_string(status.target_state),
      antidrone_turret::to_string(status.action),
      antidrone_turret::to_string(status.trigger_state),
      status.distance_m,
      status.confidence);

    if (status.trigger_state == antidrone_turret::TriggerState::kRequested) {
      request_trigger(target);
    }
  }

  void on_actuator_status(const ActuatorStatus& actuator_status)
  {
    last_actuator_state_ = actuator_status.state == ActuatorStatus::READY
                             ? antidrone_turret::ActuatorState::kReady
                             : antidrone_turret::ActuatorState::kReloading;
  }

  // Поки перший /actuator/status не надійшов, актуатор вважається неготовим:
  // краще пропустити епізод, ніж запитати постріл наосліп.
  [[nodiscard]] antidrone_turret::ActuatorState actuator_state() const
  {
    return last_actuator_state_.value_or(antidrone_turret::ActuatorState::kReloading);
  }

  void request_trigger(const antidrone_turret::TargetInput& target)
  {
    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s is not available yet", kActuatorTriggerService);
      return;
    }

    // Цілі приходять частіше, ніж статус актуатора, тому стан локально
    // переводиться у RELOADING одразу. Без цього наступний кадр тієї ж цілі
    // встиг би надіслати другий запит пострілу до приходу RELOADING.
    last_actuator_state_ = antidrone_turret::ActuatorState::kReloading;

    auto request = std::make_shared<TriggerActuator::Request>();
    request->confidence = target.confidence;
    request->distance_m = target.distance_m;

    trigger_client_->async_send_request(
      request,
      [this](rclcpp::Client<TriggerActuator>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(),
          "trigger accepted=%s trigger_count=%u",
          response->accepted ? "true" : "false",
          response->trigger_count);
      });
  }

  antidrone_turret::DecisionConfig config_{};

  rclcpp::Subscription<TargetTrack>::SharedPtr target_track_subscription_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_status_subscription_;
  rclcpp::Publisher<GimbalCmd>::SharedPtr gimbal_cmd_publisher_;
  rclcpp::Publisher<ServoCmd>::SharedPtr servo_cmd_publisher_;
  rclcpp::Publisher<TurretStatus>::SharedPtr turret_status_publisher_;
  rclcpp::Client<TriggerActuator>::SharedPtr trigger_client_;

  std::optional<antidrone_turret::ActuatorState> last_actuator_state_ = std::nullopt;
};

int main(int argc, char** argv)
{
  std::cout << "hello from turret_controller_node" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}
