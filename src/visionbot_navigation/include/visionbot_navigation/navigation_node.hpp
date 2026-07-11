#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

enum class NavState 
{ 
  WAITING_FOR_TASK,
  EXPLORING,
  BACKING_UP,
  TURNING, 
  APPROACHING
};

class NavigationNode : public rclcpp::Node
{
  public:
    NavigationNode();

  private:
    void control_loop();
    void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void uss_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void target_callback(const std_msgs::msg::String::SharedPtr msg);

    const vision_msgs::msg::Detection2D * find_target() const;
    void publish_status(const std::string & text);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr uss_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    NavState state_{NavState::WAITING_FOR_TASK};
    std::string target_class_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    double uss_min_range_ = 12.0;

    double wall_follow_speed_ = 0.9;
    double rotation_speed_ = 0.95;
    double obstacle_distance_ = 0.5;
    double approach_distance_ = 0.8;
    double approach_speed_ = 0.1;
    double camera_center_x_ = 640.0;
    double angular_gain_ = 0.003;

    double backup_duration_ = 3;
    double turning_duration_ = 10;
    double turn_duration_ = 0.8;
    double backup_speed_ = 0.1;
    
    rclcpp::Time escape_start_time_;
};