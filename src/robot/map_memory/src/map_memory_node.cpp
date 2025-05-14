#include "map_memory_node.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

MapMemoryNode::MapMemoryNode() : Node("mapping_node"), map_memory_(this->get_logger()) {
  map_memory_.resolution_ = 0.4;
  map_memory_.length_ = 80;
  map_memory_.distance_threshold_ = 1.5;
  map_memory_.publish_rate_ = 3000;

  map_memory_.pose_.position.x = -16.0;
  map_memory_.pose_.position.y = -16.0;
  map_memory_.pose_.orientation.w = 1.0;

  map_memory_.last_x_ = std::numeric_limits<double>::quiet_NaN();
  map_memory_.last_y_ = std::numeric_limits<double>::quiet_NaN();

  // Initialize global map
  map_memory_.global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map_memory_.global_map_->info.resolution = map_memory_.resolution_;
  map_memory_.global_map_->info.width = map_memory_.length_;
  map_memory_.global_map_->info.height = map_memory_.length_;
  map_memory_.global_map_->info.origin = map_memory_.pose_;
  map_memory_.global_map_->data.assign(map_memory_.length_ * map_memory_.length_, 0);

  // Subscriptions
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::CostmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(map_memory_.publish_rate_),
    std::bind(&MapMemoryNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized");
}

void MapMemoryNode::CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  bool all_zero = std::all_of(msg->data.begin(), msg->data.end(),
                              [](int8_t val) { return val == 0; });
  if (all_zero) {
    RCLCPP_INFO(this->get_logger(), "Received costmap with all zero values.");
    return;
  }

  // Check distance
  if (!std::isnan(map_memory_.last_x_)) {
    double dist = std::hypot(map_memory_.x - map_memory_.last_x_, map_memory_.y - map_memory_.last_y_);
    if (dist < map_memory_.distance_threshold_) {
      return;
    }
  }

  map_memory_.last_x_ = map_memory_.x;
  map_memory_.last_y_ = map_memory_.y;

  // Integrate local costmap into global map
  double local_res = msg->info.resolution;
  double local_origin_x = msg->info.origin.position.x;
  double local_origin_y = msg->info.origin.position.y;
  unsigned int local_w = msg->info.width;
  unsigned int local_h = msg->info.height;

  double cos_t = std::cos(map_memory_.robot_theta_);
  double sin_t = std::sin(map_memory_.robot_theta_);

  for (unsigned int j = 0; j < local_h; ++j) {
    for (unsigned int i = 0; i < local_w; ++i) {
      int8_t occ_val = msg->data[j * local_w + i];
      if (occ_val < 0) continue;

      // Local cell center
      double lx = local_origin_x + (i + 0.5) * local_res;
      double ly = local_origin_y + (j + 0.5) * local_res;

      // Transform to world frame
      double wx = map_memory_.x + (lx * cos_t - ly * sin_t);
      double wy = map_memory_.y + (lx * sin_t + ly * cos_t);

      // Convert to global map indices
      double gx_f = (wx - map_memory_.pose_.position.x) / map_memory_.resolution_;
      double gy_f = (wy - map_memory_.pose_.position.y) / map_memory_.resolution_;
      int gx = static_cast<int>(gx_f);
      int gy = static_cast<int>(gy_f);

      if (gx < 0 || gy < 0 || gx >= map_memory_.length_ || gy >= map_memory_.length_)
        continue;

      int idx = gy * map_memory_.length_ + gx;
      int8_t& global_val = map_memory_.global_map_->data[idx];

      int current_global_cost = (global_val < 0) ? 0 : global_val;
      int merged_cost = std::max(current_global_cost, static_cast<int>(occ_val));

      global_val = static_cast<int8_t>(merged_cost);
    }
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  map_memory_.x = msg->pose.pose.position.x;
  map_memory_.y = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  map_memory_.robot_theta_ = quaternionToYaw(qx, qy, qz, qw);
}

void MapMemoryNode::timerCallback() {
  auto map_msg = map_memory_.global_map_;
  map_msg->header.stamp = this->now();
  map_msg->header.frame_id = "sim_world";
  grid_pub_->publish(*map_msg);
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}