#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_core/controller.hpp"

#include "visionbot_planning/path_handler.hpp"
#include "visionbot_planning/pure_pursuit.hpp"

namespace visionbot_motion
{
  class MotionPlannerNode : public nav2_core::Controller
  {
    public:
      MotionPlannerNode() = default;
      ~MotionPlannerNode() = default;

      void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
      void cleanup() override;
      void activate() override;
      void deactivate() override;

      geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & robot_pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;
      void setPlan(const nav_msgs::msg::Path & path) override;
      void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    private:
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

      std::string plugin_name_;
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
      rclcpp::Logger logger_{rclcpp::get_logger("MotionPlannerNode")};
      rclcpp::Clock::SharedPtr clock_;
      rclcpp_lifecycle::LifecycleNode::WeakPtr lifecycle_node_;
      std::shared_ptr<tf2_ros::Buffer> tf_;
      nav_msgs::msg::Path global_plan_;

      PurePursuit controller_;
      PathHandler path_handler_;

      std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
        const tf2_ros::Buffer & tf_buffer,
        const std::string & target_frame,
        const std::string & source_frame) const;
      std::string robot_base_frame_{"base_link"};
  };
};
