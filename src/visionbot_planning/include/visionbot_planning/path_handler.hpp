#pragma once

#include <cmath>
#include <optional>
#include <cstddef>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace visionbot_motion
{
  class PathHandler
  {
    public:
      PathHandler() = default;

      void setPath(const nav_msgs::msg::Path & path);
      void clearPath();
      [[nodiscard]] bool hasPath() const;
      [[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped> & getPoses();

      [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> getLookaheadPoint(
        const geometry_msgs::msg::PoseStamped & robot_pose, double lookahead_distance);

    private:
      nav_msgs::msg::Path path_;
      size_t last_closest_index_{0};

      [[nodiscard]] static double calculateDistance(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2);
  };
};
