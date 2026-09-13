#pragma once

#include "detection_window.hpp"
#include "ground_truth.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
      void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
      void latencyCallback(const std_msgs::msg::Float32::SharedPtr msg);

      void flushWindow(const DetectionWindow & window);
      void updateVisibility(double robot_x, double robot_y, double robot_yaw);

      rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr latency_sub_;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      std::array<DetectionWindow, 8> windows_;
      std::ofstream csv_;
      std::string model_name_;
      float last_inference_ms_{0.0f};
  };
};
