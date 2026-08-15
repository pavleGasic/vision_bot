#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/visionbot_control/heartbeat.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace visionbot_control
{
  HeartbeatPubNode::HeartbeatPubNode() : Node("heartbeat_pub_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "heartbeat", 10
    );
    timer_ = this->create_wall_timer(10s,
      std::bind(&HeartbeatPubNode::publishHeartbeat, this)
    );
  }

  void HeartbeatPubNode::publishHeartbeat()
  {
    auto message = std_msgs::msg::String();
    message.data = "Alive";

    publisher_->publish(message);
  }
}

int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<visionbot_control::HeartbeatPubNode>());
    rclcpp::shutdown();

    return 0;
  }
