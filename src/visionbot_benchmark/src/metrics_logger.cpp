#include "../include/visionbot_benchmark/metrics_logger.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace visionbot_benchmark
{

  static rclcpp::QoS sensor_qos()
  {
    return rclcpp::QoS(1).best_effort();
  }

  MetricsLogger::MetricsLogger() : Node("metrics_logger_node")
  {
    declare_parameter<std::string>("model_name", "");
    declare_parameter<std::string>("output_dir", "/tmp/visionbot_benchmark");

    model_name_ = get_parameter("model_name").as_string();
    if (model_name_.empty())
    {
      RCLCPP_FATAL(get_logger(), "Parameter 'model_name' is required");
      throw std::runtime_error("Parameter 'model_name' is required");
    }

    const auto output_dir = get_parameter("output_dir").as_string();
    std::filesystem::create_directories(output_dir);

    const auto path = output_dir + "/" + model_name_ + "_" + std::to_string(std::time(nullptr)) + ".csv";
    csv_.open(path);
    if (!csv_.is_open()) {
      RCLCPP_FATAL(get_logger(), "Failed to open CSV file: %s", path.c_str());
      throw std::runtime_error("Failed to open CSV file: " + path);
    }

    csv_ << "model_name,gt_object_id,coco_name,detected,max_confidence,detection_count,frames_visible,inference_ms_avg,entry_x,entry_y\n";

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", sensor_qos(), std::bind(&MetricsLogger::odomCallback, this, std::placeholders::_1));
    latency_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/inference_latency_ms", 10, std::bind(&MetricsLogger::latencyCallback, this, std::placeholders::_1));
    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections", 10, std::bind(&MetricsLogger::detectionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MetricsLogger initialized. Logging to: %s", path.c_str());
  }

  MetricsLogger::~MetricsLogger()
  {
    for (auto & window : windows_) {
      if (window.active) flushWindow(window);
    }
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
      RCLCPP_INFO(get_logger(), "MetricsLogger CSV file closed.");
    }
  }

  void MetricsLogger::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    robot_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    updateVisibility(robot_x_, robot_y_, robot_yaw_);
  }

  void MetricsLogger::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    for (auto & window : windows_)
    {
      if (window.active) {
        window.recordFrame(last_inference_ms_);
        for (const auto & detection : msg->detections) {
          if (detection.results.empty()) continue;
          if (detection.results[0].hypothesis.class_id == window.ground_truth->coco_name) {
            window.recordDetection(detection.results[0].hypothesis.score);
          }
        }
      }
    }
  }

  void MetricsLogger::latencyCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_inference_ms_ = msg->data;
  }

  void MetricsLogger::flushWindow(const DetectionWindow & window)
  {
    csv_ << model_name_ << ","
         << window.ground_truth->id << ","
         << window.ground_truth->coco_name << ","
         << (window.detected() ? "true" : "false") << ","
         << std::fixed << std::setprecision(4) << window.max_confidence << ","
         << static_cast<int>(window.detection_count) << ","
         << static_cast<int>(window.frames_visible) << ","
         << std::setprecision(2) << window.avgInferenceMs() << ","
         << window.entry_x << ","
         << window.entry_y
         << "\n";
    csv_.flush();
  }

  void MetricsLogger::updateVisibility(double robot_x, double robot_y, double robot_yaw)
  {
    constexpr double FOV_HALF_RAD = 0.8901;

    for (size_t i = 0; i < GROUND_TRUTH_OBJECTS.size(); ++i)
    {
      const auto & gt = GROUND_TRUTH_OBJECTS[i];
      auto & window = windows_[i];

      const double dx = gt.x - robot_x;
      const double dy = gt.y - robot_y;
      const double distance = std::hypot(dx, dy);
      const double angle_to_object = std::atan2(dy, dx);

      double angle_diff = angle_to_object - robot_yaw;
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

      const bool visible = distance <= gt.max_range_m && std::abs(angle_diff) <= FOV_HALF_RAD;

      if (visible && !window.active) {
        window.open(gt, robot_x, robot_y);
        RCLCPP_INFO(get_logger(), "Object '%s' entered visibility zone", gt.id.c_str());
      } else if (!visible && window.active) {
        RCLCPP_INFO(get_logger(), "Object '%s' left visibility zone", gt.id.c_str());
        flushWindow(window);
        window.close();
      }
    }
  }
}

int main (int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<visionbot_benchmark::MetricsLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
