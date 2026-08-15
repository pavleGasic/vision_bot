#include "visionbot_planning/motion_planner_node.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace visionbot_motion 
{
  MotionPlannerNode::MotionPlannerNode(const rclcpp::NodeOptions & options)
    : Node("motion_planner_node", options)
  {
    PurePursuitParams init_params;
    init_params.lookahead_dist = declare_parameter<double>("lookahead_distance", 0.5);
    init_params.max_linear_vel = declare_parameter<double>("max_linear_velocity", 0.7);
    init_params.min_linear_vel = declare_parameter<double>("min_linear_velocity", 0.05);
    init_params.max_angular_vel = declare_parameter<double>("max_angular_velocity", 0.7);
    init_params.k_curvature = declare_parameter<double>("k_curvature", 0.5);

    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.1);

    controller_.updateParams(init_params);

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&MotionPlannerNode::onParametersChange, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/astar/path", 10, std::bind(&MotionPlannerNode::pathCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/target_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MotionPlannerNode::controlLoop, this));
  }

  void MotionPlannerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    if (path->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received path is empty!");
      return;
    }

    std::lock_guard<std::mutex> lock(path_mutex_);
    path_handler_.setPath(*path);
  }

  void MotionPlannerNode::controlLoop()
  {
    std::lock_guard<std::mutex> lock(path_mutex_);

    if (!path_handler_.hasPath()) {
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;

    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        path_handler_.getFrameId(), robot_base_frame_, tf2::TimePointZero);

      robot_pose.header = tf_stamped.header;
      robot_pose.pose.position.x = tf_stamped.transform.translation.x;
      robot_pose.pose.position.y = tf_stamped.transform.translation.y;
      robot_pose.pose.position.z = tf_stamped.transform.translation.z;
      robot_pose.pose.orientation = tf_stamped.transform.rotation;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup error: %s", ex.what());
      return;
    }

    if (path_handler_.isGoalReached(robot_pose, goal_tolerance_)) {
      RCLCPP_INFO(get_logger(), "Target goal reached successfully.");
      stopRobot();
      path_handler_.clearPath();
      return;
    }

    auto target_pose_opt = path_handler_.getLookaheadPoint(robot_pose, controller_.getParams().lookahead_dist);

    if (!target_pose_opt.has_value()) {
      RCLCPP_WARN(get_logger(), "Failed to locate target lookahead pose.");
      stopRobot();
      return;
    }

    auto target_pose = target_pose_opt.value();
    target_pose_pub_->publish(target_pose);

    geometry_msgs::msg::PoseStamped target_in_robot_frame;
    try {
      const auto tf_to_robot = tf_buffer_->lookupTransform(
        robot_base_frame_, target_pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(target_pose, target_in_robot_frame, tf_to_robot);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Transform error to robot frame: %s", ex.what());
      stopRobot();
      return;
    }

    auto cmd_vel_data = controller_.computeVelocity(
      target_in_robot_frame.pose.position.x, target_in_robot_frame.pose.position.y);

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = cmd_vel_data.linear;
    cmd_msg.angular.z = cmd_vel_data.angular;
    cmd_pub_->publish(cmd_msg);
  }

  void MotionPlannerNode::stopRobot()
  {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);
  }

  rcl_interfaces::msg::SetParametersResult MotionPlannerNode::onParametersChange(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto current_params = controller_.getParams();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
    if (param.get_name() == "lookahead_distance") {
      current_params.lookahead_dist = param.as_double();
    } else if (param.get_name() == "max_linear_velocity") {
      current_params.max_linear_vel = param.as_double();
    } else if (param.get_name() == "min_linear_velocity") {
      current_params.min_linear_vel = param.as_double();
    } else if (param.get_name() == "max_angular_velocity") {
      current_params.max_angular_vel = param.as_double();
    } else if (param.get_name() == "k_curvature") {
      current_params.k_curvature = param.as_double();
    } else if (param.get_name() == "goal_tolerance") {
      goal_tolerance_ = param.as_double();
    }
  }

  controller_.updateParams(current_params);
  return result;
  }
}

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<visionbot_motion::MotionPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}