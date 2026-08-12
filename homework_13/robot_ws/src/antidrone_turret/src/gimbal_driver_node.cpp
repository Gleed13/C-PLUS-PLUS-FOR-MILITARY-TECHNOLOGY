#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/gimbal_command.hpp"

#include "antidrone_turret/target_decision_logic.hpp"

namespace {

constexpr auto kQoSHistoryDepth = 10;
constexpr auto kGimbalCmdTopic = "/gimbal/cmd";

const char* direction_name(const std::int8_t direction)
{
  return antidrone_turret::to_string(
    static_cast<antidrone_turret::GimbalDirection>(direction));
}

}  // namespace

// Драйвер вертикального наведення. У цьому ДЗ реальним залізом не керує:
// приймає типізовану команду і логує, що вона дійшла.
class GimbalDriverNode final : public rclcpp::Node {
public:
  using GimbalCmd = antidrone_turret::msg::GimbalCommand;

  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    subscription_ = create_subscription<GimbalCmd>(
      kGimbalCmdTopic,
      kQoSHistoryDepth,
      [this](const GimbalCmd& command) {
        on_gimbal_command(command);
      });

    RCLCPP_INFO(get_logger(), "gimbal driver listening on %s", kGimbalCmdTopic);
  }

private:
  void on_gimbal_command(const GimbalCmd& command)
  {
    RCLCPP_INFO(
      get_logger(),
      "gimbal_driver_node отримав: direction=%s target_y=%.1f error_y=%.1f",
      direction_name(command.direction),
      command.target_y,
      command.error_y);
  }

  rclcpp::Subscription<GimbalCmd>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  std::cout << "hello from gimbal_driver_node" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}
