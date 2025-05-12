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


#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State
    {
        WAITING_FOR_GOAL,              // Robot is waiting for a goal
        WAITING_FOR_ROBOT_TO_REACH_GOAL // Robot is moving to the goal
    };

    State current_state_;

    
    robot::PlannerCore planner_;
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_; 


    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    void publishPath()
    void checkGoalAndReplan()

    // Data storage
    nav_msgs::msg::OccupancyGrid latest_map_;
    geometry_msgs::msg::Point current_pose_;
    geometry_msgs::msg::Point goal_pose_


    // Directions for BFS exploration (up, down, left, right)
    std::vector<std::pair<int, int>> directions_;

    // BFS helper arrays
    std::vector<std::vector<bool>> visited_;
    std::vector<std::vector<std::pair<int, int>>> parent_;

    // BFS helper methods
    bool isFree(int x, int y);
    std::vector<geometry_msgs::msg::Point> bfs(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &goal);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);


};


#endif 
