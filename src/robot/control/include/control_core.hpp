#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(const nav_msgs::msg::Path::SharedPtr current_path_,
                                                                      const nav_msgs::msg::Odometry::SharedPtr robot_odom_);
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Odometry::SharedPtr robot_odom_,
                                              const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

  private:
    rclcpp::Logger logger_;
  
    double lookahead_distance_ = 1.0;  // Lookahead distance
    double linear_speed_ = 0.5;       // Constant forward speed
};

} 

#endif 
