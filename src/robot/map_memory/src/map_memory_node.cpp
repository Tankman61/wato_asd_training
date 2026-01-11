#include "map_memory_node.hpp"
#include <chrono>

MapMemoryNode::MapMemoryNode() 
  : Node("map_memory"),
    map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 
    10, 
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 
    10, 
    std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1)
  );
  
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MapMemoryNode::updateMap, this)
  );
  
  RCLCPP_INFO(this->get_logger(), "Map Memory Node initialized");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received costmap");
  map_memory_.storeCostmap(msg);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received odometry data");
  map_memory_.updateRobotPose(msg);
}

void MapMemoryNode::updateMap() {
  RCLCPP_DEBUG(this->get_logger(), "Updating map");
  auto global_map = map_memory_.getGlobalMap();
  if (global_map) {
    map_pub_->publish(*global_map);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
