#pragma once

#include "ground_truth.hpp"

#include <string>

namespace visionbot_benchmark
{
  struct DetectionWindow
  {
    const GroundTruth* ground_truth{nullptr};

    bool active{false};
    int8_t frames_visible{0};
    int8_t detection_count{0};
    float max_confidence{0.0f};
    float inference_ms_sum{0.0f};
    double entry_x{0.0};
    double entry_y{0.0};
    std::string fp_class{};
    float fp_conf{0.0f};

    void open(const GroundTruth & gt, double robot_x, double robot_y);
    void close();
    void recordFrame(float inference_ms);
    void recordDetection(float confidence);
    void recordFalsePositive(const std::string & class_id, float confidence);

    float avgInferenceMs() const;
    bool detected() const { return detection_count > 0; }
  };
}
