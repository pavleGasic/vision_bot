#include "../include/visionbot_benchmark/benchmark_runner.hpp"
#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

namespace visionbot_benchmark
{
  BenchmarkRunner::BenchmarkRunner() : Node("benchmark_runner_node"), current_waypoint_index_(0)
  {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    waypoints_ = {
      {0.0, 0.0, 0.0, "Start"},
      {-0.85, -4.1, 1, "living room [couch, visitor_kid]"},
      {6.8, -1.6, 0.0, "kitchen appliances [frigde, oven]"},
      {6.8, -1.6, 1.57, "kitchen [chairs_kitchen]"},
      {3.6, 1.7, 1.57, "balcony [person_standing, bicycle]"},
      {0.2, 2.1, 1.57, "balcony [chairs_balcony, vase]"},
      {-4.46, 1.3, 1.57, "bedroom [bed, laptop]"},
      {-7.8, 1, 1.57, "bedroom [bed, laptop]"},
      {-4.3, -2.5, -1.57, "office [suitcase, couch_bedroom]"},
      {-6.6, -2.3, 3.14, "office [chair_office, backpack]"},
      {0.0, 0.0, 0.0, "Home"},
    };

    timer_ = this->create_wall_timer(1s, [this]() {
      if (!action_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(this->get_logger(), "Action server not available, waiting...");
        return;
      }
      timer_->cancel();
      sendNextWaypoint();
    });
  }

  void BenchmarkRunner::sendNextWaypoint()
  {
    if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
      RCLCPP_INFO(this->get_logger(), "All waypoints have been sent.");
      return;
    }

    const auto & waypoint = waypoints_[current_waypoint_index_];
    RCLCPP_INFO(this->get_logger(), "Preparing to send waypoint: %s -> (%.2f, %.2f)",
                waypoint.label.c_str(),
                waypoint.x, waypoint.y);

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = waypoint.x;
    goal.pose.pose.position.y = waypoint.y;
    goal.pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Successfully reached waypoint.");
            current_waypoint_index_++;
            sendNextWaypoint();
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Waypoint navigation was aborted, moving to next waypoint.");
            current_waypoint_index_++;
            sendNextWaypoint();
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Waypoint navigation was canceled.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
      };

    action_client_->async_send_goal(goal, send_goal_options);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visionbot_benchmark::BenchmarkRunner>());
  rclcpp::shutdown();
  return 0;
}
