#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

	nav_msgs::msg::OccupancyGrid processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    rclcpp::Logger logger_;

    // map settings
    double resolution_ = 0.1;
    int width_ = 100;
    int height_ = 100;
    double origin_x_ = -5.0;
    double origin_y_ = -5.0;

};

}  

#endif  