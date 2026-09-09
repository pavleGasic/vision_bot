#include "visionbot_benchmark/detection_window.hpp"

namespace visionbot_benchmark
{
  void DetectionWindow::open(const GroundTruth & gt, double robot_x, double robot_y)
  {
    ground_truth = &gt;
    active = true;
    frames_in_zone = 0;
    detection_count = 0;
    max_confidence = 0.0f;
    inference_ms_sum = 0.0f;
    entry_x = robot_x;
    entry_y = robot_y;
  }

  void DetectionWindow::close()
  {
    active = false;
  }

  void DetectionWindow::recordFrame(float inference_ms)
  {
    ++frames_in_zone;
    inference_ms_sum += inference_ms;
  }

  void DetectionWindow::recordDetection(float confidence)
  {
    ++detection_count;
    if (confidence > max_confidence) {
      max_confidence = confidence;
    }
  }

  float DetectionWindow::avgInferenceMs() const
  {
    if (frames_in_zone == 0) {
      return 0.0f;
    }
    return inference_ms_sum / static_cast<float>(frames_in_zone);
  }
}
