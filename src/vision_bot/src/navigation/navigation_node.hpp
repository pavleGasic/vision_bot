#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

enum class NavState { WAITING_FOR_TASK, EXPLORING, APPROACHING };

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
    float uss_min_range_{12.0f};

    double wall_follow_speed_;
    double rotation_speed_;
    double obstacle_distance_;
    double approach_distance_;
    double approach_speed_;
    double camera_center_x_;
    double angular_gain_;
};