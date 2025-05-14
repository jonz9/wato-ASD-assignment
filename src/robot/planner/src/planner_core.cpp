#include "planner_core.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace planner {

// heuristic function to estimate absolute distance between current and goal cells
double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

// checks if cell is valid (within bounds and not an obstacle)
bool PlannerCore::isCellValid(const nav_msgs::msg::OccupancyGrid &map, int x, int y) {
  if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) || y >= static_cast<int>(map.info.height))
    return false;
  int idx = y * map.info.width + x;
  return map.data[idx] >= 0 && map.data[idx] < 50;
}

// converts world coordinates to map cell index
CellIndex PlannerCore::worldToMap(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy) {
  int mx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int my = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(mx, my);
}

geometry_msgs::msg::PoseStamped PlannerCore::mapToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = map.info.origin.position.x + (idx.x + 0.5) * map.info.resolution;
  pose.pose.position.y = map.info.origin.position.y + (idx.y + 0.5) * map.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::Path PlannerCore::aStarPath(
  const nav_msgs::msg::OccupancyGrid &map,
  const geometry_msgs::msg::Pose &start_pose,
  const geometry_msgs::msg::Point &goal_point)
{
  CellIndex start = worldToMap(map, start_pose.position.x, start_pose.position.y);
  CellIndex goal = worldToMap(map, goal_point.x, goal_point.y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  open_set.emplace(start, 0.0);
  g_score[start] = 0.0;

  std::vector<CellIndex> directions = {{1,0},{-1,0},{0,1},{0,-1}};

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      // Reconstruct path
      nav_msgs::msg::Path path;
      path.header = map.header;
      path.header.stamp = rclcpp::Clock().now();

      for (CellIndex idx = goal; idx != start; idx = came_from[idx])
        path.poses.push_back(mapToWorld(map, idx));
      path.poses.push_back(mapToWorld(map, start));
      std::reverse(path.poses.begin(), path.poses.end());
      return path;
    }

    for (const auto &dir : directions) {
      CellIndex neighbor(current.x + dir.x, current.y + dir.y);
      if (!isCellValid(map, neighbor.x, neighbor.y)) continue;

      double tentative_g = g_score[current] + 1.0;
      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.emplace(neighbor, f);
      }
    }
  }

  return nav_msgs::msg::Path(); // Empty path if goal not reached
}

}
