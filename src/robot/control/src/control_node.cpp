// #include "control_core.hpp"
// #include "control_node.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "tf2/LinearMath/Matrix3x3.h"
// #include "tf2/LinearMath/Quaternion.h"

// ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
//   control_.lookahead_distance_ = 1.0;
//   control_.max_steering_angle_ = 0.5;
//   control_.steering_gain_ = 1.5;
//   control_.goal_tolerance_ = 0.1;
//   control_.linear_velocity_ = 0.5;

//   path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
//     "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  
//   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//     "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

//   cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//   timer_ = this->create_wall_timer(
//     std::chrono::milliseconds(100), [this]() { controlLoop(); });
// }

// // control loop
// void ControlNode::controlLoop() {
//   if (!current_path_ || !robot_odom_) {
//     return;
//   }

//   auto lookahead_point = findLookaheadPoint();
//   if (!lookahead_point) {
//       return;
//   }

//   auto cmd_vel = computeVelocity(*lookahead_point);

//   cmd_vel_pub_->publish(cmd_vel);
// }

// std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
//   for (const auto & pose : current_path_->poses) {
//     double distance = computeDistance(pose.pose.position, robot_odom_->pose.pose.position);
//     if (distance > control_.lookahead_distance_) {
//       return pose;
//     }
//   }
//   return std::nullopt;
// }

// geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
//   geometry_msgs::msg::Twist cmd;
//   double distance = computeDistance(target.pose.position, control_.robot_odom_->pose.pose.position);
//   if (distance < control_.goal_tolerance_) {
//     cmd.linear.x = 0.0;
//     cmd.angular.z = 0.0;
//   } else {
//     cmd.linear.x = control_.linear_velocity_;
//     double angle_to_target = std::atan2(target.pose.position.y - control_.robot_odom_->pose.pose.position.y,
//                                          target.pose.position.x - control_.robot_odom_->pose.pose.position.x);
//     double angle_diff = angle_to_target - extractYaw(control_.robot_odom_->pose.pose.orientation);
//     cmd.angular.z = control_.steering_gain_ * angle_diff;
//   }
//   return cmd;
// }

// double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
//   return std::hypot(a.x - b.x, a.y - b.y);
// }

// double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
//   tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
//   double roll, pitch, yaw;
//   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//   return yaw;
// }

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ControlNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}