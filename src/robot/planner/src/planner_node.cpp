#include "planner_node.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include <queue>
// #include <algorithm>

// #include <queue>
// #include <unordered_map>
// #include <cmath>
// #include <functional>


PlannerNode::PlannerNode() : Node("planner"), current_state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger())) {
  // Subscribers 
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

}


void PlannerNode::planPath() {
    // Your existing path planning logic (currently not implemented in the code)
}


void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
          planPath();
    }
}


void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = msg->point;
    goal_received_ = true;
    current_state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_position_ = msg->pose.pose.position;
}

void PlannerNode::publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
    nav_msgs::msg::Path path_msg;  
    path_msg.header.stamp = this->get_clock()->now();  
    path_msg.header.frame_id = "map"; 
    path_msg.poses = poses; 
    path_pub_->publish(path_msg);
}

void PlannerNode::checkGoalAndReplan() {
    if (current_state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        double distance = std::sqrt(std::pow(robot_position_.x - goal_.x, 2) + std::pow(robot_position_.y - goal_.y, 2));
        if (distance < 0.5) {
            current_state_ = State::WAITING_FOR_GOAL;
        }
    }
}

bool PlannerNode::isFree(int x, int y) {
    if (x >= 0 && x < latest_map_.info.width && y >= 0 && y < latest_map_.info.height) {
        return latest_map_.data[y * latest_map_.info.width + x] == 0; // 0 means free space
    }
    return false; 
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
