#include "control_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}


double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt((a.x-b.x) * (a.x-b.x) + (a.y-b.y) * (a.y-b.y));
}
 
double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // yoinked from aubree's code cause it's 3am
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(const nav_msgs::msg::Path::SharedPtr current_path_,
                                                                  const nav_msgs::msg::Odometry::SharedPtr robot_odom_) {

  // assume current_path_ && robot_odom_
  
  auto &currPoint = robot_odom_->pose.pose.position;
  
  if (current_path_->poses.empty()) {
      return std::nullopt;
  }

  std::optional<geometry_msgs::msg::PoseStamped> lookaheadPoint = std::nullopt;

  for(auto &curr : current_path_->poses){
    auto &point = curr.pose.position;
    double dist = computeDistance(currPoint, point);
    if(dist >= lookahead_distance_){ // want to go to this
      lookaheadPoint = curr;
      break;
    }
  }

  // If we didn't find a point far enough away, use the last point (goal)
  if (!lookaheadPoint) {
      lookaheadPoint = current_path_->poses.back();
  }

  return lookaheadPoint;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Odometry::SharedPtr robot_odom_,
                                          const geometry_msgs::msg::PoseStamped &target) {

  // --- Get steering angle ---
  // angle = target - yaw
  auto &currPoint = robot_odom_->pose.pose.position;
  auto &targetPoint = target.pose.position;
  // angle between curr point and target point = tan^{-1}(dy / dx)
  double targetAngle = atan2(targetPoint.y - currPoint.y, targetPoint.x - currPoint.x);
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);
  double steeringAngle = targetAngle - yaw;

  // Normalize angle to [-pi, pi]
  while (steeringAngle > M_PI) steeringAngle -= 2.0 * M_PI;
  while (steeringAngle < -M_PI) steeringAngle += 2.0 * M_PI;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;

  // --- Calculate angle velocity and store as Twist ---
  // curvature = 2sin(steering angle)/distance
  double distance = computeDistance(currPoint, targetPoint);
  distance = std::max(distance, 0.5);  // Prevent division by tiny values

  cmd_vel.angular.z = 2 * std::sin(steeringAngle) / distance;
  return cmd_vel;
}



}  
