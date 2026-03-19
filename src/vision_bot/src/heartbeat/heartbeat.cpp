#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "heartbeat.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

HeartbeatPubNode::HeartbeatPubNode() : Node("heartbeat_pub_node")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>(
    "heartbeat", 10
  );
  timer_ = this->create_wall_timer(10s,
    std::bind(&HeartbeatPubNode::publish_heartbeat, this)
  );
}

void HeartbeatPubNode::publish_heartbeat()
{
  auto message = std_msgs::msg::String();
  message.data = "Alive";

  publisher_->publish(message);
}

int main(int argc, char * argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatPubNode>());
  rclcpp::shutdown();

  return 0;
}