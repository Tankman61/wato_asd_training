#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  // Place callback function here
  void publishMessage();

private:
  robot::CostmapCore costmap_;
  // Place these constructs here

  // subscriber (shared pointer)
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

   // Callback function (runs every time with a new laser scan)
   void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif