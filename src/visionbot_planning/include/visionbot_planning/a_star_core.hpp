#pragma once

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cstdint>

namespace visionbot_planning
{
  struct GridNode
  {
    int x{0};
    int y{0};
    double g_cost{0.0};
    double h_cost{0.0};

    double f_cost() const { return g_cost + h_cost; }

    bool operator>(const GridNode & other) const
    {
      return f_cost() > other.f_cost();
    }

    bool operator==(const GridNode & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  class AStarCore
  {
    public:
      AStarCore() = default;

      std::vector<GridNode> plan(
        const std::vector<int8_t> & grid_data,
        uint32_t width,
        uint32_t height,
        const GridNode & start,
        const GridNode & goal,
        std::vector<int8_t> & visited_visualization
      );

      bool is_in_bounds(const GridNode & node, uint32_t width, uint32_t height) const;
      uint32_t node_to_index(const GridNode & node, uint32_t width) const;

    private:
      double calculate_manhattan(const GridNode & node, const GridNode & goal) const;
  };
};
