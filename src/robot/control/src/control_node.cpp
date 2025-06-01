#include <optional>
#include "control_core.hpp"
#include "control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  control_.lookahead_distance_ = 1.0;
  control_.max_steering_angle_ = 0.5;
  control_.steering_gain_ = 1.5;
  control_.goal_tolerance_ = 0.01;
  control_.linear_velocity_ = 0.5;

  // subscribers and publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

// path subscription callback
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
  goal_reached_ = false;
}

// odom subscription callback
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

// find look ahead point for the robot using the current path
std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    return std::nullopt;
  }

  for (const auto & pose : current_path_->poses) {
    double distance = computeDistance(pose.pose.position, robot_odom_->pose.pose.position);
    if (distance > control_.lookahead_distance_) {
      return pose;
    }
  }

  return current_path_->poses.back(); // return last point if no lookahead point found
}

// control loop
void ControlNode::controlLoop() {
  // Check if we have a valid path and odometry data
  if (goal_reached_ || !current_path_ || !robot_odom_) {
    return;
  }

  auto lookahead_point = findLookaheadPoint();

  // if reached end of the path or no lookahead point found, stop the robot, reset the path
  if (!lookahead_point) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_pub_->publish(stop_cmd);

    goal_reached_ = true;
    current_path_.reset();
    return;
  }

  auto cmd_vel = this->computeVelocity(*lookahead_point);
  cmd_pub_->publish(cmd_vel);
}

// computer distance between point a and point b
double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// extract yaw from quaternion by converting it to roll, pitch, yaw
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

// compute velocity command based on the lookahead point
geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd;
  double distance = computeDistance(target.pose.position, robot_odom_->pose.pose.position);

  if (distance < control_.goal_tolerance_) {
    goal_reached_ = true;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    current_path_.reset(); // reset the path after reaching the goal
    return cmd; // stop the robot
  }

  cmd.linear.x = control_.linear_velocity_;
  double angle_to_target = std::atan2(target.pose.position.y - robot_odom_->pose.pose.position.y,
                                        target.pose.position.x - robot_odom_->pose.pose.position.x);
  double angle_diff = angle_to_target - extractYaw(robot_odom_->pose.pose.orientation);
  cmd.angular.z = control_.steering_gain_ * angle_diff;

  return cmd;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}