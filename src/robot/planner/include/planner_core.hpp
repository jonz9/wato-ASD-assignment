#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <unordered_map>
#include <queue>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

// structs for A* algorithm
// represent a cell's index in the grid
struct CellIndex {
  int x, y;
  CellIndex(int xx = 0, int yy = 0) : x(xx), y(yy) {}
  bool operator==(const CellIndex &other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex &other) const { return !(*this == other); }
};

// represent a hash function for CellIndex
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// node
struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// comparison function
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

namespace planner {

class PlannerCore {
public:
  static nav_msgs::msg::Path aStarPath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &start_pose,
    const geometry_msgs::msg::Point &goal_point
  );

private:
  static double heuristic(const CellIndex &a, const CellIndex &b);
  static bool isCellValid(const nav_msgs::msg::OccupancyGrid &map, int x, int y);
  static CellIndex worldToMap(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy);
  static geometry_msgs::msg::PoseStamped mapToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx);
};

}

#endif