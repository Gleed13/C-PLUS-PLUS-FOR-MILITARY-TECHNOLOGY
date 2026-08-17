// Керуюча нода: усе відбувається у відповідь на /robot/local_scan, без таймерів.
#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mission_explorer/map_memory.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/state_qos.hpp"

namespace {

using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto kScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kTriggerService = "/payload/trigger";

constexpr auto kContactType = "C";

}  // namespace

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer")
  {
    const auto event_qos = rclcpp::QoS{10}.reliable();

    move_pub_ = create_publisher<MoveCommand>(kMoveTopic, event_qos);
    status_pub_ = create_publisher<StudentStatus>(kStatusTopic, event_qos);
    trigger_client_ = create_client<PayloadTrigger>(kTriggerService);

    // QoS має збігатися з transient_local публікатором світу, інакше
    // сканування ніколи не прийде і робот не зрушить з місця.
    scan_sub_ = create_subscription<LocalScan>(
      kScanTopic, underground_world::make_state_qos(), [this](const LocalScan::SharedPtr msg) { on_local_scan(*msg); });
  }

private:
  void publish_status(const std::uint8_t state)
  {
    StudentStatus msg;
    msg.state = state;
    status_pub_->publish(msg);
  }

  void on_local_scan(const LocalScan& scan)
  {
    for (const auto& cell : scan.cells) {
      map_.observe(cell.x, cell.y, cell.cell_type.empty() ? '#' : cell.cell_type.front(), cell.contact_id);
    }
    const mission_explorer::P robot{scan.robot_x, scan.robot_y};
    map_.set_robot(robot);

    // (1) Контакти важливіші за рух.
    for (const auto& cell : scan.cells) {
      if (cell.cell_type != kContactType || engaged_.count(cell.contact_id) != 0) {
        continue;
      }
      if (!trigger_client_->service_is_ready() && !trigger_client_->wait_for_service(std::chrono::seconds{2})) {
        RCLCPP_WARN(get_logger(), "payload trigger service is not available yet");
        return;
      }

      engaged_.insert(cell.contact_id);
      publish_status(StudentStatus::ENGAGING);

      auto request = std::make_shared<PayloadTrigger::Request>();
      request->contact_id = cell.contact_id;
      request->x = cell.x;
      request->y = cell.y;

      // Тільки асинхронно: синхронне очікування всередині callback - це spin у spin.
      trigger_client_->async_send_request(request, [this, id = cell.contact_id](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(get_logger(), "trigger contact_id=%d accepted=%s reason=%s", id, response->accepted ? "true" : "false",
                    response->reason.c_str());
      });

      // Рух у цьому ж callback гонитиметься з move_commit_period_ms і дасть duplicate_triggers.
      return;
    }

    // (2) Рух до найближчого фронтиру.
    try {
      const auto step = map_.next_step();
      if (!step.has_value()) {
        publish_status(StudentStatus::DONE);
        return;
      }

      MoveCommand command;
      command.direction = mission_explorer::delta_to_direction(robot, *step);
      publish_status(StudentStatus::EXPLORING);
      move_pub_->publish(command);
    }
    catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "planning failed: %s", error.what());
      publish_status(StudentStatus::FAILED);
    }
  }

  mission_explorer::MapMemory map_;
  std::set<int> engaged_;
  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
