#include "costmap_core.hpp"

namespace robot {
  CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}
  nav_msgs::msg::OccupancyGrid CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    auto grid = nav_msgs::msg::OccupancyGrid();
    (void)scan; // we're not using scan yet
    return grid;
  }
}