#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("mapping_node"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  map_memory_.last_x_ = 0.0;
  map_memory_.last_y_ = 0.0;
  map_memory_.distance_threshold_ = 5.0;
  map_memory_.costmap_updated_ = false;
  map_memory_.should_update_map_ = false;

  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the latest costmap
  map_memory_.latest_costmap_ = *msg;
  map_memory_.costmap_updated_ = true;
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - map_memory_.last_x_, 2) + std::pow(y - map_memory_.last_y_, 2));
  if (distance >= map_memory_.distance_threshold_) {
    map_memory_.last_x_ = x;
    map_memory_.last_y_ = y;
    map_memory_.should_update_map_ = true;
  }
}

// Timer-based map update
void MapMemoryNode::updateMap() {
  if (map_memory_.should_update_map_ && map_memory_.costmap_updated_) {
    integrateCostmap();
    grid_pub_->publish(map_memory_.global_map_);
    map_memory_.should_update_map_ = false;
  }
}

// Integrate the latest costmap into the global map
void integrateCostmap() {
    // Transform and merge the latest costmap into the global map
    // (Implementation would handle grid alignment and merging logic)
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
