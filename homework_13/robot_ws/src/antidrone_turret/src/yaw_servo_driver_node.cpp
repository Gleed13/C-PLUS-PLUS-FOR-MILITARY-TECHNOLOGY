#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/servo_command.hpp"

#include "antidrone_turret/target_decision_logic.hpp"

namespace {

constexpr auto kQoSHistoryDepth = 10;
constexpr auto kServoCmdTopic = "/servo/cmd";

const char* direction_name(const std::int8_t direction)
{
  return antidrone_turret::to_string(
    static_cast<antidrone_turret::ServoDirection>(direction));
}

}  // namespace

// Драйвер горизонтального повороту турелі. У цьому ДЗ реальним залізом не
// керує: приймає типізовану команду і логує, що вона дійшла.
class YawServoDriverNode final : public rclcpp::Node {
public:
  using ServoCmd = antidrone_turret::msg::ServoCommand;

  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    subscription_ = create_subscription<ServoCmd>(
      kServoCmdTopic,
      kQoSHistoryDepth,
      [this](const ServoCmd& command) {
        on_servo_command(command);
      });

    RCLCPP_INFO(get_logger(), "yaw servo driver listening on %s", kServoCmdTopic);
  }

private:
  void on_servo_command(const ServoCmd& command)
  {
    RCLCPP_INFO(
      get_logger(),
      "yaw_servo_driver_node отримав: direction=%s target_x=%.1f error_x=%.1f",
      direction_name(command.direction),
      command.target_x,
      command.error_x);
  }

  rclcpp::Subscription<ServoCmd>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  std::cout << "hello from yaw_servo_driver_node" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
