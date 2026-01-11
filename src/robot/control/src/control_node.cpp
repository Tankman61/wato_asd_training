#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters

  goal_tolerance_ = 0.1;     // Distance to consider the goal reached
  

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_) {
      return;
    }
    // Find the lookahead point
    auto lookahead_point = control_.findLookaheadPoint(current_path_, robot_odom_);
    if (!lookahead_point) {
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
