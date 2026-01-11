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
  std::vector<int> obstacle_indices;
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
      if (grid.data[index] != 100) {
          grid.data[index] = 100;
          obstacle_indices.push_back(index);
      }
    }
  }

  // 5. Inflate Obstacles
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
  for (int obs_index : obstacle_indices) {
      int obs_x = obs_index % width_;
      int obs_y = obs_index / width_;

      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
              int neighbor_x = obs_x + dx;
              int neighbor_y = obs_y + dy;

              if (neighbor_x >= 0 && neighbor_x < width_ && neighbor_y >= 0 && neighbor_y < height_) {
                  double distance = std::hypot(dx, dy) * resolution_;
                  if (distance < inflation_radius_) {
                      int neighbor_index = neighbor_y * width_ + neighbor_x;
                      double cost = 100.0 * (1.0 - distance / inflation_radius_);
                      if (static_cast<int8_t>(cost) > grid.data[neighbor_index]) {
                          grid.data[neighbor_index] = static_cast<int8_t>(cost);
                      }
                  }
              }
          }
      }
  }

  return grid;
}

}