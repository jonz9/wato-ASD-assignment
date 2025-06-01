#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    double x, y;
    double last_x_, last_y_;
    double robot_theta_;
    double resolution_;
    double length_;
    geometry_msgs::msg::Pose pose_;
    double distance_threshold_;
    int publish_rate_;
    bool costmap_updated_;
    bool initialized_ = false;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  private:
    rclcpp::Logger logger_;
};

}  

#endif  