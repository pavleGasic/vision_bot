#include "navigation_node.hpp"
#include <limits>
#include <algorithm>

NavigationNode::NavigationNode() : Node("navigation_node")
{
  wall_follow_speed_ = 0.15;
  rotation_speed_ = 0.4;
  obstacle_distance_ = 0.5;
  approach_distance_ = 0.8;
  approach_speed_ = 0.1;
  camera_center_x_ = 640.0;
  angular_gain_ = 0.003;

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/diff_drive_controller/cmd_vel_unstamped", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(
    "/navigation/status", 10);
  
  detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections", 10, std::bind(&NavigationNode::detections_callback, this, std::placeholders::_1));
  uss_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/ultrasonic/front_raw", 10, std::bind(&NavigationNode::uss_callback, this, std::placeholders::_1));
  target_sub_ = create_subscription<std_msgs::msg::String>(
    "/navigation/target_class", 10, std::bind(&NavigationNode::target_callback, this, std::placeholders::_1));
  
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NavigationNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Navigation node ready, send target to '/navigation/target_class'");
}

void NavigationNode::detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) 
{
  latest_detections_ = msg;
}

void NavigationNode::uss_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if(msg->ranges.empty()) return;
  uss_min_range_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
}

void NavigationNode::target_callback(const std_msgs::msg::String::SharedPtr msg)
{
  target_class_ = msg->data;
  state_ = NavState::EXPLORING;
  RCLCPP_INFO(get_logger(), "New target: '%s', starting exploration...", target_class_.c_str());
}

const vision_msgs::msg::Detection2D * NavigationNode::find_target() const 
{
  if (!latest_detections_ || target_class_.empty()) return nullptr;

  const vision_msgs::msg::Detection2D * best = nullptr;
  double best_score = 0.0;

  for (const auto & detection : latest_detections_->detections) {
    for (const auto & hyp : detection.results) {
      if (hyp.hypothesis.class_id == target_class_ && hyp.hypothesis.score > best_score) {
        best_score = hyp.hypothesis.score;
        best = &detection;
      }
    }
  }
  
  return best;
}

void NavigationNode::control_loop()
{
  geometry_msgs::msg::Twist cmd;

  switch (state_)
  {
    case NavState::WAITING_FOR_TASK:
      publish_status("WAITING - send target to /navigation/target_class");
      break;
    
    case NavState::EXPLORING: {
      const auto * target = find_target();
      if (target) {
        state_ = NavState::APPROACHING;
        RCLCPP_INFO(get_logger(), "Target '%s' found, approaching...", target_class_.c_str());
        break;
      }
      if (uss_min_range_ > obstacle_distance_) {
        cmd.linear.x = wall_follow_speed_;
        cmd.angular.z = 0.1;
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = rotation_speed_;
      }
      publish_status("EXPLORING, target: " + target_class_);
      break;
    }

    case NavState::APPROACHING: {
      const auto * target = find_target();
      if (!target) {
        state_ = NavState::EXPLORING;
        RCLCPP_WARN(get_logger(), "Target loss, resuming exploration...");
        break;
      }

      if (uss_min_range_ <= approach_distance_) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        state_ = NavState::WAITING_FOR_TASK;
        RCLCPP_INFO(get_logger(), "Reached target '%s', waiting for new task", target_class_.c_str());
        target_class_.clear();
        break;
      }

      double error_x = target->bbox.center.position.x - camera_center_x_;
      cmd.linear.x = approach_speed_;
      cmd.angular.z = -error_x * angular_gain_;
      publish_status("APPROACHING, target: " + target_class_);
      break;
    }

    cmd_vel_pub_->publish(cmd);
  }
}

void NavigationNode::publish_status(const std::string & text) 
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}