#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "costmap_core.hpp"
#include "costmap_node.hpp" 
#include "rclcpp/rclcpp.hpp"

CostmapNode::CostmapNode() : Node("costmap_node"),
  costmap_(rclcpp::get_logger("CostmapNode"))
{
  costmap_.resolution_ = 0.1;
  costmap_.max_cost_ = 100;
  costmap_.pose_.position.x = -5.0;
  costmap_.pose_.position.y = -5.0;
  costmap_.pose_.orientation.w = 1.0;
  costmap_.length_ = 100;
  costmap_.inflation_radius_ = 1.5;
  costmap_.occupancy_grid_.resize(costmap_.length_, std::vector<int>(costmap_.length_, -1));

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap() {
  for (auto &row : costmap_.occupancy_grid_) {
    std::fill(row.begin(), row.end(), -1);
  }
}

void CostmapNode::convertToGrid(double range, double angle, int &x, int &y) {
  double x_world = range * std::cos(angle);
  double y_world = range * std::sin(angle);

  x = static_cast<int>((x_world - costmap_.pose_.position.x) / costmap_.resolution_);
  y = static_cast<int>((y_world - costmap_.pose_.position.y) / costmap_.resolution_);
}

void CostmapNode::markObstacle(int x, int y) {
  if (x >= 0 && y >= 0 && x < costmap_.length_ && y < costmap_.length_) {
    costmap_.occupancy_grid_[y][x] = costmap_.max_cost_;
  }
}

void CostmapNode::inflateObstacle(int x_grid, int y_grid) {
  int radius_cells = static_cast<int>(costmap_.inflation_radius_ / costmap_.resolution_);

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      int nx = x_grid + dx;
      int ny = y_grid + dy;
      if (nx >= 0 && ny >= 0 && nx < costmap_.length_ && ny < costmap_.length_) {
        double distance = std::hypot(dx * costmap_.resolution_, dy * costmap_.resolution_);
        if (distance <= costmap_.inflation_radius_) {
          int new_cost = static_cast<int>(costmap_.max_cost_ * (1.0 - distance / costmap_.inflation_radius_));
          costmap_.occupancy_grid_[ny][nx] = std::max(costmap_.occupancy_grid_[ny][nx], new_cost);
        }
      }
    }
  }
}

void CostmapNode::publishCostmap(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header = scan->header; 

  grid_msg.info.resolution = costmap_.resolution_;
  grid_msg.info.width = costmap_.length_;
  grid_msg.info.height = costmap_.length_;
  grid_msg.info.origin.position.x = costmap_.pose_.position.x;
  grid_msg.info.origin.position.y = costmap_.pose_.position.y;
  grid_msg.info.origin.orientation.w = costmap_.pose_.orientation.w;
  grid_msg.data.resize(costmap_.length_ * costmap_.length_, 0);
  for (int y = 0; y < costmap_.length_; ++y) {
    for (int x = 0; x < costmap_.length_; ++x) {
      double cost = costmap_.occupancy_grid_[y][x];
      grid_msg.data[y * costmap_.length_ + x] = cost > 0 ? cost : 0;
    }
  }

  costmap_pub_->publish(grid_msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range > scan->range_min && range < scan->range_max) {
      int x, y;
      convertToGrid(range, angle, x, y);
      markObstacle(x, y);
      inflateObstacle(x, y);

      // if (x >= 0 && x < costmap_.length_ &&
      //     y >= 0 && y < costmap_.length_) {
      //   int index = y * costmap_.length_ + x;
      //   costmap_.occupancy_grid_[y][x] = 100;

      //   inflateObstacle(x, y);
      // }
    }
  }
  publishCostmap(scan);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}