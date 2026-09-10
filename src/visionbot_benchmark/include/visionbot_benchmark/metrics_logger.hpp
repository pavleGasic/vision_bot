#pragma once

#include "detection_window.hpp"
#include "ground_truth.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include <string>
#include <array>
#include <fstream>

namespace visionbot_benchmark
{
  class MetricsLogger : public rclcpp::Node
  {
    public:
      MetricsLogger();
      ~MetricsLogger();

    private:
      void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
      void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
      void latencyCallback(const std_msgs::msg::Float32::SharedPtr msg);

      void flushWindow(const DetectionWindow & window);
      void updateZones(double robot_x, double robot_y);

      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
      rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr latency_sub_;

      std::array<DetectionWindow, 8> windows_;
      std::ofstream csv_;
      std::string model_name_;
      double robot_x_{0.0};
      double robot_y_{0.0};
      float last_inference_ms_{0.0f};
  };
};
