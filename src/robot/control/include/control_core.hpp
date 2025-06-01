#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double goal_tolerance_;
    double linear_velocity_;
  
  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::Path path_; 
};

} 

#endif 