#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace robot {

// 1. Grid Index Struct
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

// 2. Node for A* Search
struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// 3. Comparator for Min-Heap
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score; // Smallest f_score at the top
  }
};

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger& logger);

  // The main entry point for planning
  // Note: I renamed 'calculatePath' to 'planPath' to match what we wrote in the node
  nav_msgs::msg::Path planPath(const nav_msgs::msg::Odometry &start_odom, 
                               const geometry_msgs::msg::PointStamped &goal_point,
                               const nav_msgs::msg::OccupancyGrid &map);

private:
  rclcpp::Logger logger_;

  // Helper: Convert world (meters) to grid index
  CellIndex worldToGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid &map);
  
  // Helper: Convert grid index to world (meters)
  geometry_msgs::msg::PoseStamped gridToWorld(int gx, int gy, const nav_msgs::msg::OccupancyGrid &map);

  // Helper: Distance heuristic (Euclidean)
  double heuristic(CellIndex a, CellIndex b);

  // Helper: Check if index is valid and free
  bool isValid(int x, int y, const nav_msgs::msg::OccupancyGrid &map);
};

}

#endif