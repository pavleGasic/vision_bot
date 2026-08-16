#include "visionbot_planning/a_star_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace visionbot_planning
{
  static constexpr int8_t CELL_OCCUPIED = 100;
  static constexpr int8_t CELL_UNKNOWN = -1;
  static constexpr double MAX_OCCUPANCY_COST = 100;

  uint32_t AStarCore::nodeToIndex(const GridNode & node, uint32_t width) const
  {
    return static_cast<uint32_t>(node.y) * width + static_cast<uint32_t>(node.x);
  }

  bool AStarCore::isInBounds(const GridNode & node, uint32_t width, uint32_t height) const
  {
    return node.x >= 0 && node.x < static_cast<int>(width) &&
           node.y >= 0 && node.y < static_cast<int>(height);
  }

  double AStarCore::calculateManhattan(const GridNode & node, const GridNode & goal) const
  {
    return std::abs(node.x - goal.x) + std::abs(node.y - goal.y);
  }

  double AStarCore::calculateCellCost(int8_t raw_cost) const
  {
    double normalized = raw_cost / MAX_OCCUPANCY_COST;
    return std::exp(obstacle_cost_scale_ * normalized) - 1.0;
  }

  std::vector<GridNode> AStarCore::plan(
    nav2_costmap_2d::Costmap2D & costmap,
    const GridNode & start,
    const GridNode & goal)
  {
    std::vector<GridNode> path;
    const auto width = costmap.getSizeInCellsX();
    const auto height = costmap.getSizeInCellsY();

    if(!isInBounds(start, width, height) || !isInBounds(goal, width, height)){
      return path;
    }

    const uint32_t total_cells = width * height;
    uint32_t start_idx = nodeToIndex(start, width);
    uint32_t goal_idx = nodeToIndex(goal, width);

    std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
    std::vector<bool> closed(total_cells, false);
    std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
    std::unordered_map<uint32_t, GridNode> parent_map;

    GridNode start_node = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = calculateManhattan(start_node, goal);

    g_score[start_idx] = 0.0;
    open_set.push(start_node);

    const std::vector<std::pair<int, int>> directions = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };

    bool goal_reached = false;

    while (!open_set.empty()) {
      GridNode current = open_set.top();
      open_set.pop();

      uint32_t current_idx = nodeToIndex(current, width);

      if (closed[current_idx]) continue;
      closed[current_idx] = true;

      if (current_idx == goal_idx) {
        goal_reached = true;
        break;
      }

      for (const auto & [dx, dy] : directions) {
        GridNode neighbor{current.x + dx, current.y + dy};
        if (!isInBounds(neighbor, width, height)) continue;

        uint32_t neighbor_idx = nodeToIndex(neighbor, width);

        // skip occupied cell or cells out of map
        unsigned char raw_cost = costmap.getCost(neighbor_idx);
        if (raw_cost >= nav2_costmap_2d::LETHAL_OBSTACLE || raw_cost == nav2_costmap_2d::NO_INFORMATION) continue;

        double temp_g = g_score[current_idx] + 1.0 + calculateCellCost(raw_cost);

        if (temp_g < g_score[neighbor_idx]) {
          parent_map[neighbor_idx] = current;
          g_score[neighbor_idx] = temp_g;

          neighbor.g_cost = temp_g;
          neighbor.h_cost = calculateManhattan(neighbor, goal);
          open_set.push(neighbor);
        }
      }
    }

    if(goal_reached) {
      GridNode current = goal;
      uint32_t current_idx = goal_idx;

      while (current_idx != start_idx) {
        path.push_back(current);
        current = parent_map[current_idx];
        current_idx = nodeToIndex(current, width);
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
    }

    return path;
  }
}
