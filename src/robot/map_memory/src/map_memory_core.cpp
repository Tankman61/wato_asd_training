#include "map_memory_core.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger),
    latest_costmap_(nullptr),
    latest_odom_(nullptr),
    global_map_(nullptr)
{
  RCLCPP_INFO(logger_, "MapMemoryCore initialized");
}

double MapMemoryCore::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void MapMemoryCore::storeCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
  
  if (!latest_odom_) {
    RCLCPP_WARN(logger_, "No odometry data yet, cannot transform costmap");
    return;
  }
  
  if (!global_map_) {
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    global_map_->header.frame_id = "odom";
    global_map_->info.resolution = msg->info.resolution;
    global_map_->info.width = 1000;
    global_map_->info.height = 1000;
    global_map_->info.origin.position.x = -25.0;
    global_map_->info.origin.position.y = -25.0;
    global_map_->data.resize(1000 * 1000, -1);
    RCLCPP_INFO(logger_, "Initialized global map: %dx%d cells", 
                global_map_->info.width, global_map_->info.height);
  }
  
  double robot_x = latest_odom_->pose.pose.position.x;
  double robot_y = latest_odom_->pose.pose.position.y;
  double yaw = getYawFromQuaternion(latest_odom_->pose.pose.orientation);
  
  for (size_t i = 0; i < msg->info.width; i++) {
    for (size_t j = 0; j < msg->info.height; j++) {
      size_t costmap_index = j * msg->info.width + i;
      int8_t cell_value = msg->data[costmap_index];
      
      if (cell_value == -1) continue;
      
      double local_x = (static_cast<double>(i) - msg->info.width / 2.0) * msg->info.resolution;
      double local_y = (static_cast<double>(j) - msg->info.height / 2.0) * msg->info.resolution;
      
      double global_x = robot_x + (local_x * std::cos(yaw) - local_y * std::sin(yaw));
      double global_y = robot_y + (local_x * std::sin(yaw) + local_y * std::cos(yaw));
      
      int global_i = static_cast<int>((global_x - global_map_->info.origin.position.x) / global_map_->info.resolution);
      int global_j = static_cast<int>((global_y - global_map_->info.origin.position.y) / global_map_->info.resolution);
      
      if (global_i >= 0 && global_i < static_cast<int>(global_map_->info.width) &&
          global_j >= 0 && global_j < static_cast<int>(global_map_->info.height)) {
        size_t global_index = global_j * global_map_->info.width + global_i;
        global_map_->data[global_index] = cell_value;
      }
    }
  }
  
  global_map_->header.stamp = msg->header.stamp;
  
  RCLCPP_DEBUG(logger_, "Merged costmap into global map at position (%.2f, %.2f, yaw=%.2f)", 
               robot_x, robot_y, yaw);
}

void MapMemoryCore::updateRobotPose(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = msg;
  RCLCPP_DEBUG(logger_, "Updated robot pose: x=%.2f, y=%.2f", 
               msg->pose.pose.position.x, 
               msg->pose.pose.position.y);
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getGlobalMap() {
  if (!global_map_) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                         "No global map available yet");
    return nullptr;
  }
  
  return global_map_;
}

} 
