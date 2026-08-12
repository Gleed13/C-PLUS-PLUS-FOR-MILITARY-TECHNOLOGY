#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/actuator_status.h"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"

#include "antidrone_turret/target_decision_logic.hpp"
#include "antidrone_turret/turret_status.hpp"

namespace {

constexpr auto kQoSHistoryDepth = 10;
constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kGimbalCmdTopic = "/gimbal/cmd";
constexpr auto kServoCmdTopic = "/servo/cmd";
constexpr auto kActuatorTriggerService = "/actuator/trigger";
constexpr auto kTurretStatusTopic = "/turret/status"; //TODO: create publisher for ros2 topic echo

antidrone_turret::msg::TurretStatus to_message(const antidrone_turret::TurretStatus& status)
{
  auto message = antidrone_turret::msg::TurretStatus{};
  message.target_state = status.target_state;
  message.action = status.action;
  message.trigger_state = status.trigger_state;
  message.confidence = status.confidence;
  message.distance_m = status.distance_m;
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

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    const auto confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.8);
    const auto max_distance_m_       = declare_parameter<double>("max_distance_m", 30.0);

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
  }

private:
  void on_target_track(const TargetTrack& target_track)
  {
    antidrone_turret::TurretStatus turret_status;
    if (!target_track.visible) {
      // turret_status.target_state = 
      turret_status_publisher_->publish(to_message(turret_status));
    }
  }
  void on_actuator_status(const ActuatorStatus& actuator_status)
  {
    last_actuator_state_ = actuator_status.state;
  }

  rclcpp::Subscription<TargetTrack>::SharedPtr target_track_subscription_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_status_subscription_;
  rclcpp::Publisher<GimbalCmd>::SharedPtr gimbal_cmd_publisher_;
  rclcpp::Publisher<ServoCmd>::SharedPtr servo_cmd_publisher_;
  rclcpp::Publisher<TurretStatus>::SharedPtr turret_status_publisher_;

  std::optional<uint8_t> last_actuator_state_ = std::nullopt;
};

int main(int argc, char** argv)
{
  std::cout << "hello from turret_controller_node" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}