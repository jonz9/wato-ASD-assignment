#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class CostmapCore {
  public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double resolution_;
    double inflation_radius_;
    double max_cost_;
    geometry_msgs::msg::Pose pose_;
    int length_;

    std::vector<double> ranges_;
    std::vector<std::vector<int>> occupancy_grid_;

  private:
    rclcpp::Logger logger_;

};

}  

#endif  