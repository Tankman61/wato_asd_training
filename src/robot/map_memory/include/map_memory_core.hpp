#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void storeCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateRobotPose(const nav_msgs::msg::Odometry::SharedPtr msg);
    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap();

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);

    // Track last update position
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    bool first_update_ = true;
};

}

#endif  
