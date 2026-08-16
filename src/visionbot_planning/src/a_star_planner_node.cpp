#include "visionbot_planning/a_star_planner_node.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace visionbot_planning
{
  void AStarPlannerNode::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    lifecycle_node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(lifecycle_node_, "smooth_path");
    if (!smooth_client_->action_server_is_ready()) {
      RCLCPP_ERROR(lifecycle_node_->get_logger(), "Action server not available after waiting");
    }
  }

  void AStarPlannerNode::cleanup()
  {
    RCLCPP_INFO(lifecycle_node_->get_logger(), "Cleaning up %s of type AStarPlanner", name_.c_str());
  }

  void AStarPlannerNode::activate()
  {
    RCLCPP_INFO(lifecycle_node_->get_logger(), "Activating %s of type AStarPlanner", name_.c_str());
  }

  void AStarPlannerNode::deactivate()
  {
    RCLCPP_INFO(lifecycle_node_->get_logger(), "Deactivating %s of type AStarPlanner", name_.c_str());
  }

  nav_msgs::msg::Path AStarPlannerNode::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()>)
  {
    const auto grid_path = planner_core_.plan(
      *costmap_, worldToGrid(start.pose), worldToGrid(goal.pose));

    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;

    if (!grid_path.empty()) {
      for (const auto & node : grid_path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = global_frame_;
        pose_stamped.pose = gridToWorld(node);
        path.poses.push_back(pose_stamped);
      }
    }

    return smoothPath(path);
  }

  nav_msgs::msg::Path AStarPlannerNode::smoothPath(const nav_msgs::msg::Path & raw_path) const
  {
    nav2_msgs::action::SmoothPath::Goal goal;
    goal.path = raw_path;
    goal.check_for_collisions = false;
    goal.smoother_id = "simple_smoother";
    goal.max_smoothing_duration.sec = 10;

    auto goal_future = smooth_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      RCLCPP_WARN(lifecycle_node_->get_logger(), "Smoother goal timed out, returning raw path");
      return raw_path;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(lifecycle_node_->get_logger(), "Smoother rejected goal, returning raw path");
      return raw_path;
    }

    auto result_future = smooth_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      RCLCPP_WARN(lifecycle_node_->get_logger(), "Smoother result timed out, returning raw path");
      return raw_path;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(lifecycle_node_->get_logger(), "Smoother failed, returning raw path");
      return raw_path;
    }

    return result.result->path;
  }

  GridNode AStarPlannerNode::worldToGrid(const geometry_msgs::msg::Pose & pose) const
  {
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    return GridNode{grid_x, grid_y};
  }

  geometry_msgs::msg::Pose AStarPlannerNode::gridToWorld(const GridNode & node) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    return pose;
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(visionbot_planning::AStarPlannerNode, nav2_core::GlobalPlanner)
