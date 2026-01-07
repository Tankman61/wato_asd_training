#include "costmap_core.hpp"
#include <cmath>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::OccupancyGrid CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid grid;

  // 1. Copy Header
  grid.header = scan->header;

  // 2. Set Map Metadata
  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // 3. Initialize Grid with 0 (Free Space)
  grid.data.assign(width_ * height_, 0);

  // 4. Process Laser Points
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double range = scan->ranges[i];

    // Filter invalid ranges
    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    // Calculate Angle
    double angle = scan->angle_min + (i * scan->angle_increment);

    // Polar -> Cartesian
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Cartesian -> Grid Index
    int x_grid = static_cast<int>((x - origin_x_) / resolution_);
    int y_grid = static_cast<int>((y - origin_y_) / resolution_);

    // Check Bounds and Mark Obstacle
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
      int index = y_grid * width_ + x_grid;
      grid.data[index] = 100;
    }
  }

  return grid;
}

}