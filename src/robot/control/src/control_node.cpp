#include "control_node.hpp"

#include <cmath>
#include <optional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ControlNode::ControlNode()
  : Node("control"),
    control_(robot::ControlCore(this->get_logger())) {

  // Initialize parameters
  lookahead_distance_ = 1.0;
  goal_tolerance_ = 0.1;
  linear_speed_ = 0.5;

  // Subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
      current_path_ = msg;
    });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      robot_odom_ = msg;
    });


  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Check if data is available before using it
  if (!current_path_ || !robot_odom_) {
    return;
  }

  const auto& robot_pos = robot_odom_->pose.pose.position;
  const auto& goal_pos = current_path_->poses.back().pose.position;

  double goal_dist = computeDistance(robot_pos, goal_pos);
  if (goal_dist < goal_tolerance_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    RCLCPP_INFO(this->get_logger(), "Reached goal, stopping robot.");
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}


std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto& robot_pos = robot_odom_->pose.pose.position;

  for (const auto& pose_stamped : current_path_->poses) {
    const auto& path_pos = pose_stamped.pose.position;
    double distance = computeDistance(robot_pos, path_pos);

    if (distance >= lookahead_distance_) {
      return pose_stamped;
    }
  }

  if (!current_path_->poses.empty()) {
    return current_path_->poses.back();
  }

  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  const auto& robot_pose = robot_odom_->pose.pose;
  const auto& robot_pos = robot_pose.position;
  double robot_yaw = extractYaw(robot_pose.orientation);

  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y;

  double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  double L = std::hypot(local_x, local_y);
  if (L < 1e-6) return cmd_vel;

  double curvature = (2 * local_y) / (L * L);

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = curvature * linear_speed_;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(tf_q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
