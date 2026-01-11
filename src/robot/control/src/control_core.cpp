#include "control_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}



std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {

  // assume current_path_ && robot_odom_
  
  auto &currPoint = robot_odom_->pose.pose.position;
  auto &lookaheadPoint = std::nullopt; // edge case - account for path is empty

  for(auto &point : current_path_->pose){
    double dist = computeDistance(currPoint, point);
    if(dist >= lookahead_distance_){ // want to go to this
      lookaheadPoint = point;
      break;
    }

    // TODO: edge case - account for robot is close to final goal


  }

  RCLCPP_DEBUG(this->get_logger(), "Ran findLookaheadPoint()");
  return lookaheadPoint;
}

geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {

  // --- Get steering angle ---
  // angle = target - yaw
  auto &currPoint = robot_odom_->pose.pose.position;
  auto &targetPoint = target.pose.position;
  // angle between curr point and target point = tan^{-1}(dy / dx)
  double targetAngle = atan((currPoint.y - targetPoint.y) / (currPoint.x - targetPoint.x));
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);
  double steeringAngle = targetAngle - yaw;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_

  // --- Calculate angle velocity and store as Twist --- 
  // curvature = 2sin(steering angle)/distance

  cmd_vel.angular.z = 2 * std::sin(steeringAngle)/computeDistance(currPoint, targetPoint);

  return cmd_vel;
}
 
double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt((a.x-b.x) * (a.x-b.x) + (a.y-b.y) * (a.y-b.y));
}
 
double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // yoinked from aubree's code cause it's 3am
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}


}  
