#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // double last_x_, last_y_;
    // double distance_threshold_;
    // bool costmap_updated_;
    // bool should_update_map_;

    // nav_msgs::msg::OccupancyGrid global_map_;
    // nav_msgs::msg::OccupancyGrid latest_costmap_;

  private:
    rclcpp::Logger logger_;
};

}  

#endif  
