// #ifndef PLANNER_NODE_HPP_
// #define PLANNER_NODE_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include <vector>
// #include <utility>
// #include <cmath>


// #include "planner_core.hpp"

// class PlannerNode : public rclcpp::Node {
//   public:
//     PlannerNode();

//   private:
//     enum class State
//     {
//         WAITING_FOR_GOAL,              // Robot is waiting for a goal
//         WAITING_FOR_ROBOT_TO_REACH_GOAL // Robot is moving to the goal
//     };

//     State current_state_;


//     robot::PlannerCore planner_;
//     // Subscribers
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
//     rclcpp::TimerBase::SharedPtr timer_; 


//     // Callbacks
//     void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
//     void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
//     void publishPath()
//     void checkGoalAndReplan()

//     // Data storage
//     nav_msgs::msg::OccupancyGrid latest_map_;
//     geometry_msgs::msg::Point current_pose_;
//     geometry_msgs::msg::Point goal_pose_


//     // Directions for BFS exploration (up, down, left, right)
//     std::vector<std::pair<int, int>> directions_;

//     // BFS helper arrays
//     std::vector<std::vector<bool>> visited_;
//     std::vector<std::vector<std::pair<int, int>>> parent_;

//     // BFS helper methods
//     bool isFree(int x, int y);
//     std::vector<geometry_msgs::msg::Point> bfs(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &goal);
//     void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
//     void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

// };


// #endif 


#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void processParameters();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr  goal_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void timerCallback();

    void publishPath();
    void resetGoal();

  private:
    robot::PlannerCore planner_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap & Mutex
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::mutex map_mutex_;

    // Current goal
    geometry_msgs::msg::PointStamped current_goal_;
    bool active_goal_;
    rclcpp::Time plan_start_time_;

    // Robot odometry (x,y). For simplicity, ignoring orientation usage here.
    bool have_odom_;
    double odom_x_;
    double odom_y_;

    // Parameters
    std::string map_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    std::string path_topic_;

    double smoothing_factor_;
    int iterations_;

    double goal_tolerance_;
    double plan_timeout_;

};

#endif 