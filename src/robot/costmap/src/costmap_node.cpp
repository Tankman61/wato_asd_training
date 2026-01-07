#include <memory>

#include "costmap_node.hpp"


CostmapNode::CostmapNode() :  Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
  {
  // Initialize pub sub
  // Publisher
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Subscriber
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // process the scan
  nav_msgs::msg::OccupancyGrid grid = costmap_.processLaserScan(msg);
  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}