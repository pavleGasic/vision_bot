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

  inline const std::array<GroundTruth, 14> GROUND_TRUTH_OBJECTS = {{
    {"person_standing", "person",       0,  3.42,  4.03, 5.0},
    {"visitor_kid",     "person",       0,  0.69, -1.80, 3.0},
    {"couch",           "couch",       57,  0.90, -1.51, 4.0},
    {"refrigerator",    "refrigerator",72,  8.86, -0.71, 5.0},
    {"suitcase",        "suitcase",    28, -3.37, -4.26, 3.5},
    {"chair_office",    "chair",       56, -7.54, -3.78, 3.5},
    {"chairs_kitchen",  "chair",       56,  6.63,  0.94, 4.0},
    {"chairs_balcony",  "chair",       56, -0.02,  3.95, 3.5},
    {"bicycle",         "bicycle",      1,  2.59,  4.94, 4.0},
    {"backpack",        "backpack",    24, -8.80, -2.49, 3.0},
    {"bed",             "bed",         59, -6.02,  1.90, 4.0},
    {"laptop",          "laptop",      63, -4.64,  1.65, 2.5},
    {"oven",            "oven",        68,  8.79, -1.73, 3.5},
    {"vase",            "vase",        75, -0.17,  5.07, 2.5},
  }};
}
