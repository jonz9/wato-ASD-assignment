#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    // void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    // void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // void MapMemoryNode::updateMap();
    // void MapMemoryNode::integrateCostmap();

    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;
    
};

#endif 
