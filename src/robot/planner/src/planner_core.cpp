#include "planner_core.hpp"
#include <cmath>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {}

bool PlannerCore::isCellValid(int x, int y, const nav_msgs::msg::OccupancyGrid &map) {
  int width = map.info.width;
  int height = map.info.height;
  if (x < 0 || y < 0 || x >= width || y >= height) return false;
  int index = y * width + x;
  return map.data[index] >= 0 && map.data[index] < 50;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

geometry_msgs::msg::Pose PlannerCore::cellToPose(const CellIndex &cell, const nav_msgs::msg::OccupancyGrid &map) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = map.info.origin.position.x + cell.x * map.info.resolution;
  pose.position.y = map.info.origin.position.y + cell.y * map.info.resolution;
  pose.orientation.w = 1.0;
  return pose;
}

CellIndex PlannerCore::poseToCell(const geometry_msgs::msg::Pose &pose, const nav_msgs::msg::OccupancyGrid &map) {
  int x = static_cast<int>((pose.position.x - map.info.origin.position.x) / map.info.resolution);
  int y = static_cast<int>((pose.position.y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(x, y);
}

CellIndex PlannerCore::pointToCell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map) {
  int x = static_cast<int>((point.x - map.info.origin.position.x) / map.info.resolution);
  int y = static_cast<int>((point.y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(x, y);
}

void PlannerCore::createPlan(const nav_msgs::msg::OccupancyGrid &map,
                             const geometry_msgs::msg::Pose &start,
                             const geometry_msgs::msg::Point &goal,
                             nav_msgs::msg::Path &path,
                             const rclcpp::Time &timestamp) {
  CellIndex start_cell = poseToCell(start, map);
  CellIndex goal_cell = pointToCell(goal, map);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  open_set.emplace(start_cell, 0.0);
  g_score[start_cell] = 0.0;

  std::vector<CellIndex> directions = {{0,1},{1,0},{0,-1},{-1,0}};

  while (!open_set.empty()) {
    auto current = open_set.top().index;
    open_set.pop();

    if (current == goal_cell) break;

    for (const auto &d : directions) {
      CellIndex neighbor(current.x + d.x, current.y + d.y);
      if (!isCellValid(neighbor.x, neighbor.y, map)) continue;

      double tentative_g = g_score[current] + 1.0;
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal_cell);
        open_set.emplace(neighbor, f);
      }
    }
  }

  CellIndex current = goal_cell;
  while (came_from.find(current) != came_from.end()) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = cellToPose(current, map);
    pose_stamped.header.stamp = timestamp;
    pose_stamped.header.frame_id = map.header.frame_id;
    path.poses.push_back(pose_stamped);
    current = came_from[current];
  }
  std::reverse(path.poses.begin(), path.poses.end());
}

} // namespace robot
