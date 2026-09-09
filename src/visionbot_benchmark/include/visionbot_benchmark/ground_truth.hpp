#pragma once

#include <array>
#include <string>

namespace visionbot_benchmark
{
  struct GroundTruth
  {
    std::string name;
    double x;
    double y;
    double zone_radius;
  };

  inline constexpr double DEFAULT_ZONE_RADIUS = 1.5;

  inline const std::array<GroundTruth, 6> GROUND_TRUTH_OBJECTS = {{
    {"A", 0.0, 0.0, DEFAULT_ZONE_RADIUS},
    {"B", 5.0, 0.0, DEFAULT_ZONE_RADIUS},
    {"C", 5.0, 5.0, DEFAULT_ZONE_RADIUS},
    {"D", 0.0, 5.0, DEFAULT_ZONE_RADIUS},
    {"E", -5.0, 5.0, DEFAULT_ZONE_RADIUS},
    {"F", -5.0, 0.0, DEFAULT_ZONE_RADIUS}
  }};
}
