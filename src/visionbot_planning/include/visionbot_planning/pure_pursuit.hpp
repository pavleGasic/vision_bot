#pragma once

#include <algorithm>
#include <cmath>

namespace visionbot_motion
{
  struct CommandVelocity
  {
    double linear;
    double angular;
  };

  struct PurePursuitParams
  {
    double lookahead_dist{0.5};
    double max_linear_vel{0.5};
    double min_linear_vel{0.05};
    double max_angular_vel{0.4};
    double k_curvature{0.5};
  };

  class PurePursuit
  {
    public:
      PurePursuit() = default;
      explicit PurePursuit(const PurePursuitParams & params);

      void updateParams(const PurePursuitParams & params);
      [[nodiscard]] PurePursuitParams getParams() const;

      [[nodiscard]] double calculateCurvature(double target_x, double target_y) const;

      [[nodiscard]] CommandVelocity computeVelocity(double target_x, double target_y) const;

    private:
      PurePursuitParams params_;

      static constexpr double angular_error_threshold_ = 1; // 1rad = 60 deg
  };

};
