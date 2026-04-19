// ============================================================================
// a_star_planner.hpp — Header file
// ============================================================================
//
// A* is Dijkstra plus a heuristic. The data structures and ROS interface are
// identical to dijkstra_planner.hpp; the only algorithmic difference is that
// the priority queue is ordered by f(n) = g(n) + h(n) instead of g(n) alone.
//
// h(n) here is the Manhattan distance to the goal. On a 4-connected grid with
// unit step cost this heuristic is admissible (never over-estimates), so A*
// still returns the optimal path while expanding far fewer cells than
// Dijkstra — that is the performance win the user asked for.
//
// ============================================================================

#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>
#include <queue>
#include <set>


// ── Graph Node ──────────────────────────────────────────────────────────────
// Same layout as the Dijkstra version, plus a heuristic field.
// ────────────────────────────────────────────────────────────────────────────
struct AStarGraphNode
{
    int x;          // grid column
    int y;          // grid row
    int cost;       // g(n) — accumulated cost from start (each step = 1)
    int heuristic;  // h(n) — Manhattan distance to goal
    int parent_idx; // index in all_nodes of the previous node, -1 = start

    AStarGraphNode(int x = 0, int y = 0, int cost = 0,
                   int heuristic = 0, int parent_idx = -1)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent_idx(parent_idx) {}

    int f() const { return cost + heuristic; }
};


// ── Main Planner Node ───────────────────────────────────────────────────────
class AStarPlanning : public rclcpp::Node
{
public:
    AStarPlanning();
    ~AStarPlanning() = default;

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // NOTE: publishes on the SAME topics as the Dijkstra planner so the rest
    // of the system (controller, RViz config) needs no changes.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    nav_msgs::msg::Path plan(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal);

    // ── Heuristic ───────────────────────────────────────────────────────
    // Manhattan distance: |dx| + |dy|. Admissible on a 4-connected grid.
    int manhattan_heuristic(const AStarGraphNode& a, const AStarGraphNode& b) const;

    // ── Coordinate conversion helpers (identical to Dijkstra) ──────────
    AStarGraphNode world_to_grid(const geometry_msgs::msg::Pose& pose) const;
    geometry_msgs::msg::Pose grid_to_world(const AStarGraphNode& node) const;
    int grid_to_index(const AStarGraphNode& node) const;
    bool is_on_map(const AStarGraphNode& node) const;
};

#endif // A_STAR_PLANNER_HPP
