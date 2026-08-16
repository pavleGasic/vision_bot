#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "visionbot_planning/a_star_core.hpp"

namespace visionbot_planning
{
  class AStarPlannerNode : public nav2_core::GlobalPlanner
  {
    public:
      AStarPlannerNode() = default;
      ~AStarPlannerNode() = default;

      void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
      void cleanup() override;
      void activate() override;
      void deactivate() override;

      nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker) override;

    private:
      std::shared_ptr<tf2_ros::Buffer> tf_;
      nav2_util::LifecycleNode::SharedPtr lifecycle_node_;
      nav2_costmap_2d::Costmap2D * costmap_;
      std::string global_frame_, name_;

      rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;

      nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw_path) const;

      GridNode worldToGrid(const geometry_msgs::msg::Pose & pose) const;
      geometry_msgs::msg::Pose gridToWorld(const GridNode & node) const;

      AStarCore planner_core_;
  };
};
