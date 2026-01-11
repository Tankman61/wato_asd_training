#include "planner_core.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>

// Helper to hash CellIndex for unordered_map
struct CellIndexHash {
    std::size_t operator()(const robot::CellIndex& idx) const {
        // Simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(const nav_msgs::msg::Odometry &start_odom, 
                                          const geometry_msgs::msg::PointStamped &goal_point,
                                          const nav_msgs::msg::OccupancyGrid &map) {
    nav_msgs::msg::Path path;
    path.header.stamp = start_odom.header.stamp;
    path.header.frame_id = map.header.frame_id;

    // 1. Convert Start/Goal to Grid
    CellIndex start_idx = worldToGrid(start_odom.pose.pose.position.x, start_odom.pose.pose.position.y, map);
    CellIndex goal_idx = worldToGrid(goal_point.point.x, goal_point.point.y, map);

    // Validate
    if (!isValid(start_idx.x, start_idx.y, map) || !isValid(goal_idx.x, goal_idx.y, map)) {
        RCLCPP_WARN(logger_, "Start or Goal is out of bounds/occupied!");
        return path;
    }

    // 2. Setup A* Structures
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    // Initialize Start
    open_set.push(AStarNode(start_idx, 0.0)); // f = 0 (technically h, but start is 0)
    g_score[start_idx] = 0.0;

    // Neighbors (Up, Down, Left, Right)
    const int dx[] = {0, 0, 1, -1};
    const int dy[] = {1, -1, 0, 0};

    // 3. A* Loop
    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        // Check if we reached the goal
        if (current.index == goal_idx) {
            // Reconstruct Path (Backwards)
            std::vector<geometry_msgs::msg::PoseStamped> path_poses;
            CellIndex step = goal_idx;
            while (step != start_idx) {
                path_poses.push_back(gridToWorld(step.x, step.y, map));
                step = came_from[step];
            }
            path_poses.push_back(gridToWorld(start_idx.x, start_idx.y, map));
            
            // Reverse to get Start -> Goal
            std::reverse(path_poses.begin(), path_poses.end());
            path.poses = path_poses;
            return path;
        }

        // Expand Neighbors
        for (int i = 0; i < 4; ++i) {
            CellIndex neighbor(current.index.x + dx[i], current.index.y + dy[i]);

            // Check if valid and not a wall
            if (!isValid(neighbor.x, neighbor.y, map)) {
                continue;
            }

            // Calculate tentative G-Score
            // Distance between neighbors is always 1 cell (assuming 4-connectivity)
            double tentative_g = g_score[current.index] + 1.0; 

            // If we found a cheaper path to this neighbor
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal_idx);
                open_set.push(AStarNode(neighbor, f));
            }
        }
    }

    RCLCPP_WARN(logger_, "Failed to find a path to the goal!");
    return path; // Return empty path if failure
}

// --- Helpers ---

CellIndex PlannerCore::worldToGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid &map) {
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    double res = map.info.resolution;

    int gx = static_cast<int>((wx - origin_x) / res);
    int gy = static_cast<int>((wy - origin_y) / res);
    return CellIndex(gx, gy);
}

geometry_msgs::msg::PoseStamped PlannerCore::gridToWorld(int gx, int gy, const nav_msgs::msg::OccupancyGrid &map) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = map.header;

    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    double res = map.info.resolution;

    pose.pose.position.x = (gx * res) + origin_x;
    pose.pose.position.y = (gy * res) + origin_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0; // Default orientation
    return pose;
}

double PlannerCore::heuristic(CellIndex a, CellIndex b) {
    // Euclidean Distance
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool PlannerCore::isValid(int x, int y, const nav_msgs::msg::OccupancyGrid &map) {
    // 1. Check Bounds
    if (x < 0 || x >= static_cast<int>(map.info.width) || 
        y < 0 || y >= static_cast<int>(map.info.height)) {
        return false;
    }

    // 2. Check Collision
    int index = y * map.info.width + x;
    if (map.data[index] > 50) { // Arbitrary threshold (100 is occupied, 0 is free)
        return false; 
    }

    return true;
}

}