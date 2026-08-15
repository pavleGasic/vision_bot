#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace visionbot_control
{
  enum SafetyStopState
  {
    SAFE = 0,
    WARNING = 1,
    DANGER = 2
  };

  class SafetyStopNode : public rclcpp::Node
  {
    public:
      SafetyStopNode();

    private:
      bool is_first_laser_msg_;
      double danger_distance_;
      double warning_distance_;
      SafetyStopState state_;
      SafetyStopState prev_state_;
      const double cmd_vel_multiplier_;
      sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
      visualization_msgs::msg::MarkerArray zones_;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_teleop_pub_;

      void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
      void cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
      SafetyStopState determineState(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  };
};
