#include "visionbot_planning/motion_planner_node.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace visionbot_motion
{
  void MotionPlannerNode::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    lifecycle_node_ = parent;
    auto node = lifecycle_node_.lock();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    PurePursuitParams controller_params_;

    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.8));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".k_curvature", rclcpp::ParameterValue(0.5));

    node->get_parameter(plugin_name_ + ".lookahead_dist", controller_params_.lookahead_dist);
    node->get_parameter(plugin_name_ + ".max_linear_vel", controller_params_.max_linear_vel);
    node->get_parameter(plugin_name_ + ".min_linear_vel", controller_params_.min_linear_vel);
    node->get_parameter(plugin_name_ + ".max_angular_vel", controller_params_.max_angular_vel);
    node->get_parameter(plugin_name_ + ".k_curvature", controller_params_.k_curvature);

    controller_.updateParams(controller_params_);

    target_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/target_pose", 10);
  }

  void MotionPlannerNode::cleanup()
  {
    RCLCPP_INFO(logger_, "Cleaning up plugin MotionPlannerNode");
    target_pose_pub_.reset();
    path_handler_.clearPath();
  }

  void MotionPlannerNode::activate()
  {
    RCLCPP_INFO(logger_, "Activating plugin MotionPlannerNode");
  }

  void MotionPlannerNode::deactivate()
  {
    RCLCPP_INFO(logger_, "Deactivating plugin MotionPlannerNode");
  }

  geometry_msgs::msg::TwistStamped MotionPlannerNode::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = robot_pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if (!path_handler_.hasPath()) {
      return cmd_vel;
    }

    auto target_pose_opt = path_handler_.getLookaheadPoint(robot_pose, controller_.getParams().lookahead_dist);
    if (!target_pose_opt.has_value()) {
      RCLCPP_WARN(logger_, "Failed to locate target lookahead pose.");
      return cmd_vel;
    }

    auto target_pose = target_pose_opt.value();
    target_pose_pub_->publish(target_pose);
    const auto tf_to_robot = lookupTransform(*tf_, robot_base_frame_, target_pose.header.frame_id);
    if (!tf_to_robot.has_value()) {
      return cmd_vel;
    }

    geometry_msgs::msg::PoseStamped target_in_robot_frame;
    tf2::doTransform(target_pose, target_in_robot_frame, tf_to_robot.value());

    auto cmd_vel_data = controller_.computeVelocity(
      target_in_robot_frame.pose.position.x, target_in_robot_frame.pose.position.y);

    cmd_vel.twist.linear.x = cmd_vel_data.linear;
    cmd_vel.twist.angular.z = cmd_vel_data.angular;

    return cmd_vel;
  }

  void MotionPlannerNode::setPlan(const nav_msgs::msg::Path & path)
  {
    path_handler_.setPath(path);
  }

  void MotionPlannerNode::setSpeedLimit(const double &, const bool &) {}

  std::optional<geometry_msgs::msg::TransformStamped> MotionPlannerNode::lookupTransform(
    const tf2_ros::Buffer & tf_buffer,
    const std::string & target_frame,
    const std::string & source_frame) const
  {
    try {
      return tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "TF lookup failed [%s -> %s]: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(visionbot_motion::MotionPlannerNode, nav2_core::Controller)
