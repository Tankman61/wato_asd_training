#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // timer for checking
    rclcpp::TimerBase::SharedPtr timer_;
    // state machine
    enum class State {
      WAITING_FOR_GOAL,
      MOVING_TO_GOAL,
    };
    State state_;

    // data storage for latest messages
    nav_msgs::msg::OccupancyGrid current_map_; // Added this to store the map
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PointStamped current_goal_;
    bool goal_received_ = false;

    // callback functions
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // timer loop
    void checkPath();

    // trigger planning
    void planPath();
};

#endif 
