#include "visionbot_planning/a_star_planner_node.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace visionbot_planning
{
  AStarPlannerNode::AStarPlannerNode() : Node("a_star_node")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", map_qos, std::bind(&AStarPlannerNode::map_callback, this, std::placeholders::_1));
    pose_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&AStarPlannerNode::goal_callback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/astar/path", 10);
    visited_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/astar/visited_map", 10);
  }

  void AStarPlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
  {
    map_ = map_msg;
    visited_map_.header.frame_id = map_msg->header.frame_id;
    visited_map_.info = map_msg->info;
    visited_map_.data.assign(map_msg->info.height * map_msg->info.width, -1);
  }

  void AStarPlannerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_goal_msg)
  {
    if (!map_) {
      RCLCPP_ERROR(get_logger(), "Map not received yet. Cannot plan path.");
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_base_tf;
    try {
      map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform from %s to base_link: %s", map_->header.frame_id.c_str(), ex.what());
      return;
    };

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = map_to_base_tf.transform.translation.x;
    start_pose.position.y = map_to_base_tf.transform.translation.y;
    start_pose.orientation = map_to_base_tf.transform.rotation;

    GridNode start_node = world_to_grid(start_pose);
    GridNode goal_node = world_to_grid(pose_goal_msg->pose);

    visited_map_.data.assign(map_->info.height * map_->info.width, -1);

    auto grid_path = planner_core_.plan(
      map_->data,
      map_->info.width,
      map_->info.height,
      start_node,
      goal_node,
      visited_map_.data
    );

    visited_map_pub_->publish(visited_map_);

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = map_->header.frame_id;

    if (!grid_path.empty()) {
      for (const auto & node : grid_path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_->header.frame_id;
        pose_stamped.pose = grid_to_world(node);
        path_msg.poses.push_back(pose_stamped);
      }
      RCLCPP_INFO(get_logger(), "Path successfully planned with %zu waypoints.", path_msg.poses.size());
      path_pub_->publish(path_msg);
    } else {
      RCLCPP_WARN(get_logger(), "No valid path found to the goal!");
    }
  }

  GridNode AStarPlannerNode::world_to_grid(const geometry_msgs::msg::Pose & pose) const
  {
    int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
    int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);
    return GridNode{grid_x, grid_y};
  }

  geometry_msgs::msg::Pose AStarPlannerNode::grid_to_world(const GridNode & node) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
    pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
    return pose;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<visionbot_planning::AStarPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
