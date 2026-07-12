#include "../include/visionbot_navigation/safety_stop.hpp"

SafetyStopNode::SafetyStopNode() : Node("safety_stop_node"), state_{SAFE}, prev_state_{SAFE}
{
  danger_distance_ = 0.3;
  warning_distance_ = 0.7;
  std::string scan_topic = "/scan";
  std::string safety_stop_topic = "/safety_stop";

  laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    10,
    std::bind(&SafetyStopNode::laser_scan_callback, this, std::placeholders::_1));

  safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic, 10);
  zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/zones", 10);
  is_first_laser_msg_ = true;

  decrease_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_decrease");
  increase_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_increase");
  while (!decrease_speed_client_->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok()) 
  {
    RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available! Waiting...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  while (!increase_speed_client_->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok()) 
  {
    RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available! Waiting...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  visualization_msgs::msg::Marker warning_zone;
  warning_zone.id = 0;
  warning_zone.action = visualization_msgs::msg::Marker::ADD;
  warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
  warning_zone.scale.z = 0.001;
  warning_zone.scale.x = warning_distance_ * 2;
  warning_zone.scale.y = warning_distance_ * 2;
  warning_zone.color.r = 1.0;
  warning_zone.color.g = 0.984;
  warning_zone.color.b = 0.0;
  warning_zone.color.a = 0.5;
  zones_.markers.push_back(warning_zone);

  visualization_msgs::msg::Marker danger_zone;
  danger_zone.id = 1;
  danger_zone.action = visualization_msgs::msg::Marker::ADD;
  danger_zone.type = visualization_msgs::msg::Marker::CYLINDER;
  danger_zone.scale.z = 0.001;
  danger_zone.scale.x = danger_distance_ * 2;
  danger_zone.scale.y = danger_distance_ * 2;
  danger_zone.color.r = 1.0;
  danger_zone.color.g = 0.0;
  danger_zone.color.b = 0.0;
  danger_zone.color.a = 0.5;
  zones_.markers.push_back(danger_zone);
}

void SafetyStopNode::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  for (const auto & range : msg->ranges)
  {
    state_ = SAFE;
    if (!std::isinf(range) && range <= warning_distance_)
    {
      state_ = WARNING;

      if (range <= danger_distance_)
      {
        state_ = DANGER;
        break;
      }
    }
  }

  if (state_ != prev_state_) {
    std_msgs::msg::Bool is_safety_stop;
    switch (state_)
    {
      case SAFE:
        is_safety_stop.data = false;
        zones_.markers.at(0).color.a = 0.5;
        zones_.markers.at(1).color.a = 0.5;
        increase_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
        RCLCPP_WARN(get_logger(), "Robot state SAFE");
        break;
      
      case WARNING:
        is_safety_stop.data = false;
        zones_.markers.at(0).color.a = 1.0;
        zones_.markers.at(1).color.a = 0.5;
        decrease_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
        RCLCPP_WARN(get_logger(), "Robot state WARNING");
        break;
      
      case DANGER:
        is_safety_stop.data = true;
        zones_.markers.at(0).color.a = 1.0;
        zones_.markers.at(1).color.a = 1.0;
        RCLCPP_WARN(get_logger(), "Robot state DANGER");
        break;
    }
    prev_state_ = state_;
    safety_stop_pub_->publish(is_safety_stop);
  }

  if (is_first_laser_msg_) {
    for (auto & zone : zones_.markers) {
      zone.header.frame_id = msg->header.frame_id;
    }
    is_first_laser_msg_ = false;
  }
  zones_pub_->publish(zones_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyStopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}