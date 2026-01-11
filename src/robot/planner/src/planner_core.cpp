#include "planner_core.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

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

    RCLCPP_INFO(logger_, "Robot at world (%.2f, %.2f) -> grid (%d, %d)",
                start_odom.pose.pose.position.x, start_odom.pose.pose.position.y, start_idx.x, start_idx.y);
    RCLCPP_INFO(logger_, "Goal at world (%.2f, %.2f) -> grid (%d, %d)",
                goal_point.point.x, goal_point.point.y, goal_idx.x, goal_idx.y);
    RCLCPP_INFO(logger_, "Map: origin=(%.2f, %.2f), size=%dx%d, res=%.2f",
                map.info.origin.position.x, map.info.origin.position.y,
                map.info.width, map.info.height, map.info.resolution);

    // Validate
    // Check bounds for start (ignore collision to allow planning out of obstacles)
    if (start_idx.x < 0 || start_idx.x >= static_cast<int>(map.info.width) || 
        start_idx.y < 0 || start_idx.y >= static_cast<int>(map.info.height)) {
        RCLCPP_WARN(logger_, "Start is out of map bounds!");
        return path;
    }

    // Check bounds AND collision for goal
    // We relax the collision check for the goal too, so we can plan INTO a "warning zone"
    if (goal_idx.x < 0 || goal_idx.x >= static_cast<int>(map.info.width) || 
        goal_idx.y < 0 || goal_idx.y >= static_cast<int>(map.info.height)) {
        RCLCPP_WARN(logger_, "Goal is out of bounds!");
        return path;
    }
    
    // 2. Setup A* Structures
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    // Initialize Start
    double h = heuristic(start_idx, goal_idx);
    open_set.push(AStarNode(start_idx, h));
    g_score[start_idx] = 0.0;

    // Neighbors (8-connectivity: Up, Down, Left, Right + Diagonals)
    const int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
    const int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};
    const double costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    // 3. A* Loop
    std::unordered_set<CellIndex, CellIndexHash> closed_set; // Keep track of visited nodes
    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        // If already processed, skip
        if (closed_set.count(current.index)) {
            continue;
        }
        closed_set.insert(current.index);

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
            RCLCPP_INFO(logger_, "Path found with %zu poses.", path.poses.size());
            return path;
        }

        // Expand Neighbors
        for (int i = 0; i < 8; ++i) {
            CellIndex neighbor(current.index.x + dx[i], current.index.y + dy[i]);

            // Check if valid and not a wall
            // Allow goal node even if "occupied" (inflated)
            bool is_goal = (neighbor == goal_idx);
            if (!is_goal && !isValid(neighbor.x, neighbor.y, map)) {
                continue;
            }

            // Calculate tentative G-Score with Cost Penalty (Weighted A*)
            int neighbor_index = neighbor.y * map.info.width + neighbor.x;
            double cell_cost = std::max(0.0, static_cast<double>(map.data[neighbor_index]));
            
            // Penalize high-cost cells. 
            // 1.0 is base movement. (cell_cost / 10.0) adds penalty (0.0 to 10.0 for max cost).
            double travel_cost = costs[i] * (1.0 + (cell_cost / 10.0)); 
            
            double tentative_g = g_score[current.index] + travel_cost; 

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

    int gx = static_cast<int>(std::floor((wx - origin_x) / res));
    int gy = static_cast<int>(std::floor((wy - origin_y) / res));
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
    pose.pose.position.z = 0.5; // Raised for visibility
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
    if (map.data[index] > 50) { // Standard threshold
        return false; 
    }

    return true;
}

}