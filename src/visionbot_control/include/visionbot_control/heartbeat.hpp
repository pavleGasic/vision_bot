#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace visionbot_control
{
  class HeartbeatPubNode : public rclcpp::Node
  {
    public:
      HeartbeatPubNode();

    private:
      void publishHeartbeat();
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
  };
};
