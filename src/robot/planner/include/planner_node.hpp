#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <utility>
#include <cmath>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"



#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State
    {
        WAITING_FOR_GOAL,              // Robot is waiting for a goal
        WAITING_FOR_ROBOT_TO_REACH_GOAL, // Robot is moving to the goal
        PLANNING
    };

    State current_state_;
    State state_;

    robot::PlannerCore planner_;
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
    void checkGoalAndReplan();
    void timerCallback();
    void planPath();

    // Data storage
    nav_msgs::msg::OccupancyGrid latest_map_;
    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::Point current_pose_;
    geometry_msgs::msg::Point goal_;
    geometry_msgs::msg::Point robot_position_;

    // Directions for BFS exploration (up, down, left, right)
    std::vector<std::pair<int, int>> directions_;

    // BFS helper arrays
    std::vector<std::vector<bool>> visited_;
    std::vector<std::vector<std::pair<int, int>>> parent_;


    bool isFree(int x, int y);
    std::vector<geometry_msgs::msg::Point> a_star(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &goal);


};


#endif 