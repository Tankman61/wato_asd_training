#include <chrono>
#include <memory>

#include "odometry_spoof.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

OdometrySpoofNode::OdometrySpoofNode() : Node("odometry_spoof") {
  // Create publisher for Odometry messages
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/filtered", 10);

  // Subscribe to Gazebo's odometry directly
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/robot/odometry",
    10,
    std::bind(&OdometrySpoofNode::odometryCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(this->get_logger(), "Odometry Spoof Node initialized - subscribing to /model/robot/odometry");
}

void OdometrySpoofNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Simply republish the Gazebo odometry to /odom/filtered
  // This makes it available to other nodes that expect standard odometry topic
  odom_pub_->publish(*msg);
  
  RCLCPP_DEBUG(this->get_logger(), "Republished odometry: x=%.2f, y=%.2f", 
               msg->pose.pose.position.x, msg->pose.pose.position.y);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySpoofNode>());
  rclcpp::shutdown();
  return 0;
}
