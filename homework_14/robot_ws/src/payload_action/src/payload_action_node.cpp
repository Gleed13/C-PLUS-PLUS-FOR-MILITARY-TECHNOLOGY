// Ретранслятор корисного навантаження: сервіс -> подія знищення контакту.
// Валідацію робить world node, тому тут лише публікація EnemyDown.
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

constexpr auto kTriggerService = "/payload/trigger";
constexpr auto kEnemyDownTopic = "/payload/enemy_down";

}  // namespace

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action")
  {
    const auto event_qos = rclcpp::QoS{10}.reliable();

    enemy_down_pub_ = create_publisher<EnemyDown>(kEnemyDownTopic, event_qos);

    trigger_srv_ = create_service<PayloadTrigger>(
      kTriggerService,
      [this](const PayloadTrigger::Request::SharedPtr request, PayloadTrigger::Response::SharedPtr response) {
        on_trigger(*request, *response);
      });
  }

private:
  void on_trigger(const PayloadTrigger::Request& request, PayloadTrigger::Response& response)
  {
    EnemyDown msg;
    msg.contact_id = request.contact_id;
    msg.x = request.x;
    msg.y = request.y;
    enemy_down_pub_->publish(msg);

    response.accepted = true;
    response.reason = "engaged";

    RCLCPP_INFO(get_logger(), "engaged contact_id=%d at (%d,%d)", request.contact_id, request.x, request.y);
  }

  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<PayloadTrigger>::SharedPtr trigger_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
