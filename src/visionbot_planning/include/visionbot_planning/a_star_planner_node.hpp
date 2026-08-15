#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "visionbot_planning/a_star_core.hpp"

namespace visionbot_planning
{
  class AStarPlannerNode : public rclcpp::Node
  {
    public:
      AStarPlannerNode();

    private:
      std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
        const tf2_ros::Buffer & tf_buffer,
        const std::string & target_frame,
        const std::string & source_frame
      );

      void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
      void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_goal_msg);

      GridNode worldToGrid(const geometry_msgs::msg::Pose & pose) const;
      geometry_msgs::msg::Pose gridToWorld(const GridNode & node) const;

      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visited_map_pub_;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      nav_msgs::msg::OccupancyGrid::SharedPtr map_;
      nav_msgs::msg::OccupancyGrid visited_map_;

      AStarCore planner_core_;
  };
};
