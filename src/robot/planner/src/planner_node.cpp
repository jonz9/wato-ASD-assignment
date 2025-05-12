// #include "planner_node.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include <queue>
// #include <algorithm>

// void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
//     latest_map_ = *msg;
//     if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
//           planPath();
//       }
      
//     visited_.resize(map_.info.height, std::vector<bool>(map_.info.width, false));
//     parent_.resize(map_.info.height, std::vector<std::pair<int, int>>(map_.info.width, {-1, -1}));
// }

// void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
//     goal_ = msg->point;

//     // Transition to "waiting for robot to reach goal" state if we're waiting for a goal
//     if (current_state_ == State::WAITING_FOR_GOAL) {
//         current_state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
//     }
// }

// void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     robot_position = msg->pose.pose.position;
// }

// void PlannerNode::publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
// {
//     nav_msgs::msg::Path path_msg;  // Create the message to publish
//     path_msg.header.stamp = this->get_clock()->now();  // Add timestamp
//     path_msg.header.frame_id = "map";  // The reference frame for the path (usually "map")
//     path_msg.poses = poses;  // Add the list of poses to the message

//     // Publish the path message
//     path_pub_->publish(path_msg);
// }

// void PlannerNode::checkGoalAndReplan()
// {
//     if (current_state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
//         // Check if the robot has reached the goal or needs to replan
//         double distance = std::sqrt(std::pow(robot_position_.x - goal_.x, 2) + std::pow(robot_position_.y - goal_.y, 2));

//         if (distance < 0.5) {
//             // Goal reached, transition back to "waiting for goal"
//             current_state_ = State::WAITING_FOR_GOAL;
//         }
//     }
// }

// bool PlannerNode::isFree(int x, int y)
// {
//     // Check if the coordinates are within bounds of the map
//     if (x >= 0 && x < map_.info.width && y >= 0 && y < map_.info.height) {
//         return map_.data[y * map_.info.width + x] == 0; // 0 means free space
//     }
//     return false; // Out of bounds or blocked
// }



// PlannerNode::PlannerNode() : Node("planner"), current_state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger())) {
//   // Subscribers 
//   map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//     "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

//   goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//     "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

//   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//     "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));


//   // Publisher
//     path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

//   // Timer
//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));


//   // Initialize directions (up, down, left, right)
//     directions_ = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

//   // Initial map callback to resize visited_ and parent_ arrays
//     visited_.resize(map_.info.height, std::vector<bool>(map_.info.width, false));
//     parent_.resize(map_.info.height, std::vector<std::pair<int, int>>(map_.info.width, {-1, -1}));
// }


// std::vector<geometry_msgs::msg::Point> PlannerNode::bfs(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &goal)
// {
//     // Convert start and goal points to grid coordinates
//     int start_x = static_cast<int>(start.x);
//     int start_y = static_cast<int>(start.y);
//     int goal_x = static_cast<int>(goal.x);
//     int goal_y = static_cast<int>(goal.y);

//     // BFS setup
//     std::queue<std::pair<int, int>> q;  // Queue to store the grid cells to explore
//     q.push({start_x, start_y});
//     visited_[start_y][start_x] = true;

//     // BFS loop
//     while (!q.empty()) {
//         auto [x, y] = q.front();
//         q.pop();

//         // Check if the goal is reached
//         if (x == goal_x && y == goal_y) {
//             // Reconstruct path
//             std::vector<geometry_msgs::msg::Point> path;
//             while (parent_[y][x] != std::pair<int, int>(-1, -1)) {
//                 path.push_back(geometry_msgs::msg::Point{x, y});
//                 auto [px, py] = parent_[y][x];
//                 x = px;
//                 y = py;
//             }
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         // Explore neighbors
//         for (const auto& [dx, dy] : directions_) {
//             int new_x = x + dx;
//             int new_y = y + dy;
//             if (isFree(new_x, new_y) && !visited_[new_y][new_x]) {
//                 visited_[new_y][new_x] = true;
//                 parent_[new_y][new_x] = {x, y};  // Set parent to current node
//                 q.push({new_x, new_y});
//             }
//         }
//     }

//     return {};  // Return empty if no path is found
// }


// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PlannerNode>());
//   rclcpp::shutdown();
//   return 0;
// }


#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // load ROS2 yaml parameters
  processParameters();

  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic_,
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  // Subscribe to odometry from /odom/filtered
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

  // Timer to check goal/timeout status periodically (500 ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );

  planner_.initPlanner(smoothing_factor_, iterations_);
}

void PlannerNode::processParameters() {
  // Declare and get parameters
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("goal_topic", "/goal_pose");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<double>("smoothing_factor", 0.2);
  this->declare_parameter<int>("iterations", 20);
  this->declare_parameter<double>("goal_tolerance", 0.3);
  this->declare_parameter<double>("plan_timeout_seconds", 10.0);

  map_topic_ = this->get_parameter("map_topic").as_string();
  goal_topic_ = this->get_parameter("goal_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  path_topic_ = this->get_parameter("path_topic").as_string();
  smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
  iterations_ = this->get_parameter("iterations").as_int();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  plan_timeout_ = this->get_parameter("plan_timeout_seconds").as_double();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
  }

  // If we have an active goal, re-run plan
  if (active_goal_) {
    double elapsed = (now() - plan_start_time_).seconds();
    if (elapsed <= plan_timeout_) {
      RCLCPP_INFO(this->get_logger(), 
                  "Map updated => Replanning for current goal (time elapsed: %.2f).",
                  elapsed);
      publishPath();
    }
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg)
{
  if (active_goal_) {
    RCLCPP_WARN(this->get_logger(), "Ignoring new goal; a goal is already active.");
    return;
  }

  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "No costmap available yet. Cannot set goal.");
    return;
  }

  current_goal_ = *goal_msg;
  active_goal_ = true;
  plan_start_time_ = now();

  RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)",
              goal_msg->point.x, goal_msg->point.y);

  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  // Store the latest odometry in (odom_x_, odom_y_)
  // For simplicity, ignoring orientation or z
  odom_x_ = odom_msg->pose.pose.position.x;
  odom_y_ = odom_msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback()
{
  if (!active_goal_) {
    return;
  }

  // Check if we've timed out
  double elapsed = (now() - plan_start_time_).seconds();
  if (elapsed > plan_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Plan timed out after %.2f seconds. Resetting goal.", elapsed);
    resetGoal();
    return;
  }

  // Check if we reached the goal
  double distance = sqrt(pow(odom_x_ - current_goal_.point.x, 2) +  pow(odom_y_ - current_goal_.point.y, 2));
  if (distance < goal_tolerance_) {
    RCLCPP_WARN(this->get_logger(), "Plan succeeded! Elapsed Time: %.2f", elapsed);
    resetGoal();
    return;
  }
}

void PlannerNode::publishPath() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet. Cannot plan.");
    resetGoal();
    return;
  }

  // Use the robot's odometry as the start pose
  double start_world_x = odom_x_;
  double start_world_y = odom_y_;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if(!planner_.planPath(start_world_x, start_world_y, current_goal_.point.x, current_goal_.point.y, map_)) {
      RCLCPP_ERROR(this->get_logger(), "Plan Failed.");
      resetGoal();
      return;
    }
  }

  nav_msgs::msg::Path path_msg = *planner_.getPath();
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = map_->header.frame_id;
  
  path_pub_->publish(path_msg);
}

void PlannerNode::resetGoal() {
  active_goal_ = false;
  RCLCPP_INFO(this->get_logger(), "Resetting active goal.");

  // Publish an empty path, which should tell the robot to stop or have no path
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = this->now();

  // Use the costmap frame if available; otherwise a default like "sim_world"
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_) {
      empty_path.header.frame_id = map_->header.frame_id;
    } else {
      empty_path.header.frame_id = "sim_world";
    }
  }
  
  // Publish the empty path
  path_pub_->publish(empty_path);

  RCLCPP_INFO(this->get_logger(), "Published empty path to stop the robot.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}