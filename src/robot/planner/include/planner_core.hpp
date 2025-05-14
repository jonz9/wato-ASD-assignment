#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <unordered_map>
#include <queue>
#include <vector>

namespace robot {

struct CellIndex {
  int x, y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const CellIndex &other) const {
    return !(*this == other);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger &logger);
  void createPlan(const nav_msgs::msg::OccupancyGrid &map,
                  const geometry_msgs::msg::Pose &start,
                  const geometry_msgs::msg::Point &goal,
                  nav_msgs::msg::Path &path);

private:
  rclcpp::Logger logger_;
  bool isCellValid(int x, int y, const nav_msgs::msg::OccupancyGrid &map);
  double heuristic(const CellIndex &a, const CellIndex &b);
  geometry_msgs::msg::Pose cellToPose(const CellIndex &cell, const nav_msgs::msg::OccupancyGrid &map);
  CellIndex poseToCell(const geometry_msgs::msg::Pose &pose, const nav_msgs::msg::OccupancyGrid &map);
  CellIndex pointToCell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map);
};

} // namespace robot

#endif
