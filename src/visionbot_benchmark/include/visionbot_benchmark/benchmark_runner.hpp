#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <string>

namespace visionbot_benchmark
{
  struct Waypoint
  {
    double x, y, yaw;
    std::string label;
  };

  class BenchmarkRunner : public rclcpp::Node
  {
    public:
      BenchmarkRunner();
      ~BenchmarkRunner() = default;

    private:
      void sendNextWaypoint();

      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::vector<Waypoint> waypoints_;
      int current_waypoint_index_;
  };
};
