#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    visualization_msgs::msg::MarkerArray zones_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};