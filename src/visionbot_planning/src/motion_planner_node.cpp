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

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/astar/path", 10, std::bind(&MotionPlannerNode::pathCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/target_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MotionPlannerNode::controlLoop, this));
  }

  std::optional<geometry_msgs::msg::TransformStamped> MotionPlannerNode::lookupTransform(
    const tf2_ros::Buffer & tf_buffer,
    const std::string & target_frame,
    const std::string & source_frame
  )
  {
    try {
      return tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed [%s -> %s]: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void MotionPlannerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    if (path->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received path is empty!");
      return;
    }

    path_handler_.setPath(*path);
  }

  void MotionPlannerNode::controlLoop()
  {
    if (!path_handler_.hasPath()) {
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;

    const auto tf = lookupTransform(*tf_buffer_, path_handler_.getFrameId(), robot_base_frame_);
    if (!tf.has_value()) return;
    robot_pose.header = tf.value().header;
    robot_pose.pose.position.x = tf.value().transform.translation.x;
    robot_pose.pose.position.y = tf.value().transform.translation.y;
    robot_pose.pose.position.z = tf.value().transform.translation.z;
    robot_pose.pose.orientation = tf.value().transform.rotation;


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
    const auto tf_to_robot = lookupTransform(*tf_buffer_, robot_base_frame_, target_pose.header.frame_id);
    if (!tf_to_robot.has_value()) {
      stopRobot();
      return;
    }
    tf2::doTransform(target_pose, target_in_robot_frame, tf_to_robot.value());

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
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<visionbot_motion::MotionPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
