#include "visionbot_planning/pure_pursuit.hpp"

namespace visionbot_motion
{
  PurePursuit::PurePursuit(const PurePursuitParams & params) : params_(params) {}

  void PurePursuit::updateParams(const PurePursuitParams & params)
  {
    params_ = params;
  }

  PurePursuitParams PurePursuit::getParams() const
  {
    return params_;
  }

  double PurePursuit::calculateCurvature(double target_x, double target_y) const
  {
    const double L2 = (target_x * target_x) + (target_y * target_y);

    if (L2 < 1e-4) {
      return 0.0;
    }

    return (2.0 * target_y) / L2;
  }

  CommandVelocity PurePursuit::computeVelocity(double target_x, double target_y) const
  {
    const double curvature = calculateCurvature(target_x, target_y);

    double adaptive_linear_vel = params_.max_angular_vel / (1.0 + params_.k_curvature * std::abs(curvature));
    adaptive_linear_vel = std::clamp(adaptive_linear_vel, params_.min_linear_vel, params_.max_linear_vel);

    double angular_vel = adaptive_linear_vel * curvature;
    angular_vel = std::clamp(angular_vel, -params_.max_angular_vel, params_.max_angular_vel);

    return {adaptive_linear_vel, angular_vel};
  }
};