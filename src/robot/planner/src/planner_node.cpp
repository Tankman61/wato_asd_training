#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  state_ = State::WAITING_FOR_GOAL;

  // map sub
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  // goal sub (Fixed: Binding to goalCallback, not mapCallback)
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  // odom sub (Added this!)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // path pub (Fixed: nav_msgs::msg::Path)
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // timer every 500 ms
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::checkPath, this));
}


// callback functions

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
}

// Fixed typos in function signature
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    current_goal_ = *msg;
    goal_received_ = true;
    state_ = State::MOVING_TO_GOAL;
    
    RCLCPP_INFO(this->get_logger(), "Goal Received! Planning path...");
    planPath();
}

// Added missing checkPath implementation
void PlannerNode::checkPath() {
    if (state_ == State::MOVING_TO_GOAL) {
        // Here we could check if we are close to the goal, or replan if the map changed
        planPath();
    }
}

void PlannerNode::planPath() {
  // Fixed typo: goal_received_
  if (!goal_received_) return;

  // We need to make sure we have a map too!
  if (current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: No map received yet!");
      return;
  }

  // Call the A* algorithm in PlannerCore
  nav_msgs::msg::Path path = planner_.planPath(current_odom_, current_goal_, current_map_);
  
  // publish said path
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}