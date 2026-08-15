#include "visionbot_planning/a_star_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace visionbot_planning
{
  static constexpr int8_t CELL_OCCUPIED = 99;
  static constexpr int8_t CELL_UNKNOWN = -1;

  static constexpr int8_t VISITED_CELL_COLOR = -106;


  uint32_t AStarCore::node_to_index(const GridNode & node, uint32_t width) const
  {
    return static_cast<uint32_t>(node.y) * width + static_cast<uint32_t>(node.x);
  }

  bool AStarCore::is_in_bounds(const GridNode & node, uint32_t width, uint32_t height) const
  {
    return node.x >= 0 && node.x < static_cast<int>(width) &&
           node.y >= 0 && node.y < static_cast<int>(height);
  }

  double AStarCore::calculate_manhattan(const GridNode & node, const GridNode & goal) const
  {
    return std::abs(node.x - goal.x) + std::abs(node.y - goal.y);
  }

  double AStarCore::calculate_cell_cost(int8_t raw_cost) const
  {
    double normalized = raw_cost / 100.0;
    return std::exp(obstacle_cost_scale_ * normalized) - 1.0;
  }

  std::vector<GridNode> AStarCore::plan(
    const std::vector<int8_t> & grid_data,
    uint32_t width,
    uint32_t height,
    const GridNode & start,
    const GridNode & goal,
    std::vector<int8_t> & visited_visualization)
  {
    std::vector<GridNode> path;

    if(!is_in_bounds(start, width, height) || !is_in_bounds(goal, width, height)){
      return path;
    }

    const uint32_t total_cells = width * height;
    uint32_t start_idx = node_to_index(start, width);
    uint32_t goal_idx = node_to_index(goal, width);

    std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
    std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
    std::unordered_map<uint32_t, GridNode> parent_map;

    GridNode start_node = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = calculate_manhattan(start_node, goal);

    g_score[start_idx] = 0.0;
    open_set.push(start_node);

    const std::vector<std::pair<int, int>> directions = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };

    bool goal_reached = false;

    while (!open_set.empty()) {
      GridNode current = open_set.top();
      open_set.pop();

      uint32_t current_idx = node_to_index(current, width);
      if (current_idx == goal_idx) {
        goal_reached = true;
        break;
      }

      if (visited_visualization.size() == total_cells) {
        visited_visualization[current_idx] = VISITED_CELL_COLOR;
      }

      for (const auto & [dx, dy] : directions) {
        GridNode neighbor{current.x + dx, current.y + dy};
        if (!is_in_bounds(neighbor, width, height)) continue;

        uint32_t neighbor_idx = node_to_index(neighbor, width);

        // skip occupied cell or cells out of map
        int8_t raw_cost = grid_data[neighbor_idx];
        if (raw_cost >= CELL_OCCUPIED || raw_cost == CELL_UNKNOWN) continue;

        double temp_g = g_score[current_idx] + 1.0 + calculate_cell_cost(raw_cost);

        if (temp_g < g_score[neighbor_idx]) {
          parent_map[neighbor_idx] = current;
          g_score[neighbor_idx] = temp_g;

          neighbor.g_cost = temp_g;
          neighbor.h_cost = calculate_manhattan(neighbor, goal);
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
        current_idx = node_to_index(current, width);
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
    }

    return path;
  }
}
