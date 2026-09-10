#pragma once

#include <array>
#include <string>

namespace visionbot_benchmark
{
  struct GroundTruth
  {
    std::string id;
    std::string coco_name;
    int coco_class;
    double x;
    double y;
    double max_range_m;
  };

  inline const std::array<GroundTruth, 8> GROUND_TRUTH_OBJECTS = {{
    {"person_standing", "person", 0, 3.42, 4.03, 5.0},
    {"visitor_kid", "person", 0, 0.69, -1.80, 3.0},
    {"couch", "couch", 57, 0.90, -1.51, 4.0},
    {"refrigerator", "refrigerator", 72, 8.70, -1.03, 5.0},
    {"suitcase", "suitcase", 28, -3.37, -4.26, 3.5},
    {"chair_office", "chair", 56, -8.16, -3.62, 3.5},
    {"chairs_kitchen", "chair", 56, 6.63, 0.94, 4.0},
    {"chairs_balcony", "chair", 56, -0.53, 4.10, 3.5}
  }};
}
