#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "visionbot_planning/path_handler.hpp"
#include "visionbot_planning/pure_pursuit.hpp"

namespace visionbot_motion
{
  class MotionPlannerNode : public rclcpp::Node
  {
    public:
      explicit MotionPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
      rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
      rclcpp::TimerBase::SharedPtr control_loop_timer_;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      PurePursuit controller_;
      PathHandler path_handler_;

      std::string robot_base_frame_{"base_link"};
      double goal_tolerance_{0.1};

      std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
        const tf2_ros::Buffer & tf_buffer,
        const std::string & target_frame,
        const std::string & source_frame
      );

      void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
      void controlLoop();
      void stopRobot();
  };
};
