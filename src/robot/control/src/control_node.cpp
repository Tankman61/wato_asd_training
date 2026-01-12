#include "control_node.hpp"
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters

  goal_tolerance_ = 0.2;     // Distance to consider the goal reached
  

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { current_costmap_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

bool ControlNode::isObstacleAhead() {
  if (!current_costmap_ || !robot_odom_) return false;
  
  // Check cells in front of the robot
  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  
  // Get robot yaw
  auto& q = robot_odom_->pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  
  // Check points ahead of robot (0.5m to 1.5m ahead)
  for (double dist = 0.5; dist <= 1.5; dist += 0.2) {
    double check_x = robot_x + dist * std::cos(yaw);
    double check_y = robot_y + dist * std::sin(yaw);
    
    // Convert to costmap grid
    int gx = static_cast<int>((check_x - current_costmap_->info.origin.position.x) / current_costmap_->info.resolution);
    int gy = static_cast<int>((check_y - current_costmap_->info.origin.position.y) / current_costmap_->info.resolution);
    
    if (gx >= 0 && gx < static_cast<int>(current_costmap_->info.width) &&
        gy >= 0 && gy < static_cast<int>(current_costmap_->info.height)) {
      int idx = gy * current_costmap_->info.width + gx;
      if (current_costmap_->data[idx] > 50) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                             "Obstacle detected ahead at distance %.1f!", dist);
        return true;
      }
    }
  }
  return false;
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_) {
      return;
    }

    if (current_path_->poses.empty()) {
        stopRobot();
        return;
    }

    // Disabled obstacle check for now - needs frame alignment
    // if (isObstacleAhead()) {
    //     stopRobot();
    //     return;
    // }

    // Find the lookahead point
    auto lookahead_point = control_.findLookaheadPoint(current_path_, robot_odom_);
    if (!lookahead_point) {
        stopRobot();
        return;  // No valid lookahead point found
    }
    // Compute velocity command
    auto cmd_vel = control_.computeVelocity(robot_odom_,*lookahead_point);
    
    // Stop within goal tolerance
    if(control_.computeDistance(robot_odom_->pose.pose.position, current_path_->poses.back().pose.position) < goal_tolerance_){
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Goal reached, robot stopped.");
      return;
    }

    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}


void ControlNode::stopRobot() {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
