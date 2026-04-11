// ============================================================================
// dijkstra_planner.hpp — Header file
// ============================================================================
//
// This header declares the Dijkstraplanning node class.
// Let me explain every member and why it exists.
//
// ============================================================================

#ifndef DIJKSTRA_PLANNER_HPP
#define DIJKSTRA_PLANNER_HPP

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
// Represents a single cell in the occupancy grid during pathfinding.
// This is the C++ equivalent of the Python Graph_Node class.
//
// Why a struct and not just a pair<int,int>?
// Because we need to attach cost and a parent pointer for path reconstruction.
// The parent is stored as an index into a flat array, not a raw pointer,
// to avoid dangling pointer issues.
// ────────────────────────────────────────────────────────────────────────────
struct GraphNode
{
    int x;          // grid column
    int y;          // grid row
    int cost;       // accumulated cost from start (each step = 1)
    int parent_idx; // index in the visited array of the previous node
                    // -1 means "no parent" (this is the start node)

    GraphNode(int x = 0, int y = 0, int cost = 0, int parent_idx = -1)
        : x(x), y(y), cost(cost), parent_idx(parent_idx) {}
};


// ── Comparator for the Priority Queue ───────────────────────────────────────
// std::priority_queue is a MAX-heap by default.
// Dijkstra needs a MIN-heap (lowest cost first).
// This comparator flips the comparison: returns true when a.cost > b.cost,
// which makes the queue put the LOWEST cost on top.
//
// In Python, PriorityQueue is already a min-heap, and the __lt__ method
// on Graph_Node handled this. In C++ we need an explicit comparator.
// ────────────────────────────────────────────────────────────────────────────
struct CompareGraphNode
{
    bool operator()(const GraphNode& a, const GraphNode& b) const
    {
        return a.cost > b.cost; // ">" gives us a min-heap
    }
};


// ── Main Planner Node ───────────────────────────────────────────────────────
class Dijkstraplanning : public rclcpp::Node
{
public:
    Dijkstraplanning();
    ~Dijkstraplanning() = default;

private:
    // ── Subscriptions ───────────────────────────────────────────────────
    //
    // map_sub_: Receives the OccupancyGrid from map_server.
    //   - Published once with TRANSIENT_LOCAL durability
    //   - We must use matching QoS or we'll never receive it
    //   - The grid data is a flat int8[] array where:
    //       0   = free space (robot can go here)
    //       100 = occupied (wall/obstacle)
    //       -1  = unknown (unexplored)
    //
    // pose_sub_: Receives goal poses from RViz2's "2D Goal Pose" tool.
    //   - Each click publishes one PoseStamped message
    //   - This triggers the planning algorithm
    //
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // ── Publishers ──────────────────────────────────────────────────────
    //
    // path_pub_: Publishes the computed path as nav_msgs/Path.
    //   - A Path is just a header + vector<PoseStamped>
    //   - The controller (MPPI) subscribes to this and follows it
    //
    // map_pub_: Publishes a visualization of visited cells.
    //   - This is a copy of the map with visited cells marked
    //   - You can view this in RViz2 to watch the algorithm explore
    //   - In production, you'd remove this — it's for debugging only
    //
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // ── Map Storage ─────────────────────────────────────────────────────
    //
    // map_: Stores the latest OccupancyGrid received from map_server.
    //   - SharedPtr because the subscription callback receives a SharedPtr
    //   - We keep a reference to it so the planner can access it later
    //     when a goal arrives
    //   - nullptr until the first map message arrives
    //
    // visited_map_: A copy of the map used for visualization.
    //   - Cells are set to 10 when visited (shows as light gray in RViz2)
    //   - Reset every time a new goal is received
    //
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;

    // ── TF2 — Transform Lookup ──────────────────────────────────────────
    //
    // The planner needs to know WHERE the robot is on the map to use as
    // the start point. TF2 provides this.
    //
    // tf_buffer_: Stores recent transforms. When we call lookup_transform(),
    //   it searches this buffer for the requested transform.
    //   - unique_ptr because the Buffer has no reason to be shared
    //
    // tf_listener_: Runs in the background, automatically subscribing to
    //   /tf and /tf_static topics and filling the buffer.
    //   - shared_ptr because TransformListener stores a reference to the
    //     node internally (it needs to create a subscription)
    //   - MUST be created AFTER tf_buffer_ — it takes a reference to it
    //
    // The transform chain for your ballbot:
    //   map -> odom -> base_link
    //   AMCL publishes map->odom, your STM32 bridge publishes odom->base_link.
    //   TF2 automatically chains them to give us map->base_link.
    //
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Callbacks ───────────────────────────────────────────────────────
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // ── The Algorithm ───────────────────────────────────────────────────
    nav_msgs::msg::Path plan(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal);

    // ── Coordinate Conversion Helpers ───────────────────────────────────
    //
    // These convert between three coordinate systems:
    //
    // 1. World (meters):  What TF and goal poses use.
    //    Example: robot is at (2.35, -1.07) meters in the map frame.
    //
    // 2. Grid (cells):    Integer indices into the occupancy grid.
    //    Example: robot is in cell (147, 78).
    //
    // 3. Index (flat):    Position in the 1D data[] array.
    //    Example: cell (147, 78) is at index 78 * width + 147.
    //
    // World -> Grid:  subtract origin, divide by resolution
    // Grid -> World:  multiply by resolution, add origin
    // Grid -> Index:  y * width + x
    //
    GraphNode world_to_grid(const geometry_msgs::msg::Pose& pose) const;
    geometry_msgs::msg::Pose grid_to_world(const GraphNode& node) const;
    int grid_to_index(const GraphNode& node) const;
    bool is_on_map(const GraphNode& node) const;
};

#endif // DIJKSTRA_PLANNER_HPP