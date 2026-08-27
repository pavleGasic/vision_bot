#include "visionbot_planning/path_handler.hpp"

namespace visionbot_motion
{

  void PathHandler::setPath(const nav_msgs::msg::Path & path)
  {
    path_ = path;
    last_closest_index_ = 0;
  }

  void PathHandler::clearPath()
  {
    path_.poses.clear();
    last_closest_index_ = 0;
  }

  bool PathHandler::hasPath() const
  {
    return !path_.poses.empty();
  }

  std::vector<geometry_msgs::msg::PoseStamped> & PathHandler::getPoses()
  {
    return path_.poses;
  }

  std::optional<geometry_msgs::msg::PoseStamped> PathHandler::getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & robot_pose, double lookahead_distance)
  {
    if (!hasPath()) {
      return std::nullopt;
    }

    double min_dist = std::numeric_limits<double>::max();
    const size_t search_end = std::min(last_closest_index_ + 30, path_.poses.size());
    for (size_t i = last_closest_index_; i < search_end; ++i) {
      double dist = calculateDistance(robot_pose.pose, path_.poses[i].pose);
      if (dist < min_dist) {
        min_dist = dist;
        last_closest_index_ = i;
      }
    }

    for (size_t i = last_closest_index_; i < path_.poses.size(); ++i) {
      double dist = calculateDistance(robot_pose.pose, path_.poses[i].pose);
      if (dist >= lookahead_distance) {
        return path_.poses[i];
      }
    }

    return path_.poses.back();
  }

  double PathHandler::calculateDistance(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
  {
    return std::hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
  }
};
