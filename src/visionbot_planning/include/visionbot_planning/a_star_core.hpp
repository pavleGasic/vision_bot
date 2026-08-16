#pragma once

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cstdint>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

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
      explicit AStarCore(double obstacle_cost_scale)
        : obstacle_cost_scale_(obstacle_cost_scale) {}

      std::vector<GridNode> plan(
        nav2_costmap_2d::Costmap2D & costmap,
        const GridNode & start,
        const GridNode & goal
      );

    private:
      double obstacle_cost_scale_{5.0};
      double calculateManhattan(const GridNode & node, const GridNode & goal) const;
      double calculateCellCost(int8_t raw_cost) const;
      bool isInBounds(const GridNode & node, uint32_t width, uint32_t height) const;
      uint32_t nodeToIndex(const GridNode & node, uint32_t width) const;
  };
};
