// ============================================================================
// dijkstra_planner.cpp — Implementation
// ============================================================================

#include "ballbot_planning/dijkstra_planner.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <vector>


// ── Constructor ─────────────────────────────────────────────────────────────
//
// The constructor does four things:
//   1. Creates QoS profiles
//   2. Creates subscriptions (map + goal pose)
//   3. Creates publishers (path + visited map visualization)
//   4. Creates the TF2 buffer and listener
//
// IMPORTANT: No planning happens here. The node just sets up its
// communication channels and waits for a map and a goal.
//
// Order matters for TF2:
//   tf_buffer_ must be created BEFORE tf_listener_
//   because the listener takes a reference to the buffer.
//
// ────────────────────────────────────────────────────────────────────────────
Dijkstraplanning::Dijkstraplanning()
    : Node("dijkstra_planner")
{
    // ── QoS Profile ─────────────────────────────────────────────────────
    //
    // Why TRANSIENT_LOCAL?
    // map_server publishes the map once with TRANSIENT_LOCAL durability.
    // If our subscriber uses VOLATILE (default), and map_server published
    // the map before our node started, we'd never receive it.
    // TRANSIENT_LOCAL tells DDS: "when I connect, give me the last
    // message the publisher still has cached."
    //
    // Why RELIABLE?
    // We need every map message to arrive — a lost map message means
    // no planning at all. RELIABLE guarantees delivery with retransmission.
    //
    // depth=1: We only care about the latest map. No need to buffer
    // older versions.
    //
    rclcpp::QoS map_qos(10);
    map_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // ── Subscriptions ───────────────────────────────────────────────────
    //
    // map_sub_: When map_server publishes an OccupancyGrid, map_callback
    // stores it so the planner can use it later.
    //
    // pose_sub_: When you click "2D Goal Pose" in RViz2, it publishes a
    // PoseStamped on /goal_pose. This triggers pose_callback, which
    // looks up the robot's current position and runs the planner.
    //
    // std::bind connects the callback function to this specific instance.
    // std::placeholders::_1 is a placeholder for the message argument
    // that ROS 2 will fill in when a message arrives.
    //
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        map_qos,
        std::bind(&Dijkstraplanning::map_callback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        map_qos,
        std::bind(&Dijkstraplanning::pose_callback, this, std::placeholders::_1));

    // ── Publishers ──────────────────────────────────────────────────────
    //
    // Both use the same QoS as the subscriptions. TRANSIENT_LOCAL on the
    // path publisher means if the controller starts after the path is
    // published, it still receives it.
    //
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/dijkstra/path", map_qos);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/dijkstra/map", map_qos);

    // ── TF2 Setup ───────────────────────────────────────────────────────
    //
    // tf_buffer_ accumulates transforms from /tf and /tf_static.
    //
    // tf_listener_ is the subscriber that feeds transforms into the
    // buffer. It takes two arguments:
    //   1. The buffer to fill
    //   2. The node to create subscriptions on (this)
    //
    // Once created, it runs automatically — no manual spin needed.
    // When we later call tf_buffer_->lookupTransform(), it queries
    // the accumulated transforms.
    //
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Dijkstra planner initialized — waiting for map");
}


// ── map_callback ────────────────────────────────────────────────────────────
//
// Called every time the /map topic receives a message.
// For a static map (loaded from a file), this fires once.
// For slam_toolbox in online mode, this fires repeatedly as the map updates.
//
// We store the map and prepare the visualization grid.
//
// The OccupancyGrid message structure:
//   header: frame_id (usually "map"), timestamp
//   info:   resolution (m/pixel), width, height, origin (Pose)
//   data:   int8[] — flat array of cell values
//           0 = free, 100 = occupied, -1 = unknown
//
// ────────────────────────────────────────────────────────────────────────────
void Dijkstraplanning::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = msg;

    // Prepare the visited visualization map.
    // Copy the metadata (frame, resolution, size) but fill data with -1
    // (unknown/transparent in RViz2). We'll set cells to 10 as the
    // algorithm visits them.
    visited_map_.header.frame_id = msg->header.frame_id;
    visited_map_.info = msg->info;
    visited_map_.data.assign(
        msg->info.width * msg->info.height,
        -1  // -1 renders as transparent gray in RViz2
    );

    RCLCPP_INFO(get_logger(),
        "Map received: %d x %d, resolution: %.3f m/pixel",
        msg->info.width, msg->info.height, msg->info.resolution);
}


// ── pose_callback ───────────────────────────────────────────────────────────
//
// Called when you click "2D Goal Pose" in RViz2.
// This is the entry point for the planning pipeline:
//
//   1. Check that we have a map (can't plan without one)
//   2. Reset the visited visualization
//   3. Look up the robot's current position via TF2
//   4. Run Dijkstra's algorithm from current position to goal
//   5. Publish the resulting path
//
// ────────────────────────────────────────────────────────────────────────────
void Dijkstraplanning::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
    // ── Guard: Do we have a map? ────────────────────────────────────────
    if (!map_)
    {
        RCLCPP_WARN(get_logger(), "No map received yet — cannot plan");
        return;
    }

    // ── Reset the visited visualization ─────────────────────────────────
    // Fill with -1 so previous exploration doesn't bleed through.
    std::fill(
        visited_map_.data.begin(),
        visited_map_.data.end(),
        -1);

    // ── Look up robot position ──────────────────────────────────────────
    //
    // We ask TF2: "Where is base_link in the map frame?"
    //
    // Arguments to lookupTransform:
    //   target_frame: map_.header.frame_id (usually "map")
    //   source_frame: "base_link" (the robot's body)
    //   time:         tf2::TimePointZero means "give me the latest"
    //
    // The result is a TransformStamped containing:
    //   transform.translation.x/y/z — position
    //   transform.rotation — orientation as quaternion
    //
    // Why try/catch? The transform might not exist yet (AMCL hasn't
    // converged, or the TF tree is broken). Without the catch, the
    // node would crash.
    //
    geometry_msgs::msg::TransformStamped map_to_base;
    try
    {
        map_to_base = tf_buffer_->lookupTransform(
            map_->header.frame_id,  // target frame: "map"
            "base_link",             // source frame: robot body
            tf2::TimePointZero);     // latest available
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(get_logger(),
            "Cannot find transform map -> base_link: %s", ex.what());
        return;
    }

    // ── Extract robot pose from transform ───────────────────────────────
    //
    // TransformStamped has transform.translation (x,y,z) and
    // transform.rotation (quaternion). We copy these into a Pose
    // message because that's what our plan() function expects.
    //
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = map_to_base.transform.translation.x;
    start_pose.position.y = map_to_base.transform.translation.y;
    start_pose.orientation = map_to_base.transform.rotation;

    // ── Run the planner ─────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
        "Planning from (%.2f, %.2f) to (%.2f, %.2f)",
        start_pose.position.x, start_pose.position.y,
        goal_msg->pose.position.x, goal_msg->pose.position.y);

    auto path = plan(start_pose, goal_msg->pose);

    // ── Publish the result ──────────────────────────────────────────────
    if (!path.poses.empty())
    {
        RCLCPP_INFO(get_logger(),
            "Path found with %zu waypoints", path.poses.size());
        path_pub_->publish(path);
    }
    else
    {
        RCLCPP_WARN(get_logger(), "No path found to goal!");
    }
}


// ── plan() — Dijkstra's Algorithm ───────────────────────────────────────────
//
// This is the core algorithm. It finds the shortest path from start to goal
// on the occupancy grid.
//
// Data structures:
//
//   pending:  Priority queue (min-heap by cost).
//             Contains cells discovered but not yet fully processed.
//             Always gives us the cheapest unvisited cell.
//
//   visited:  Set of grid indices we've already processed.
//             Once a cell is in here, we never revisit it — we already
//             found the cheapest way to reach it.
//
//   all_nodes: Vector storing every GraphNode we create.
//              Parent indices point into this vector.
//              This avoids dangling pointers and lets us reconstruct
//              the path by following parent_idx chains.
//
// Algorithm:
//
//   1. Convert start and goal from world coordinates to grid cells
//   2. Put the start cell in the queue with cost 0
//   3. Loop:
//      a. Pop the lowest-cost cell from the queue
//      b. If it's the goal, stop — we found the shortest path
//      c. For each of its 4 neighbors (up/down/left/right):
//         - Skip if already visited
//         - Skip if outside the map
//         - Skip if the cell is occupied (wall)
//         - Otherwise: set cost = current + 1, record parent, enqueue
//      d. Mark the current cell as visited
//   4. Reconstruct path by following parent_idx from goal back to start
//   5. Reverse the path (it was built goal-to-start)
//   6. Convert each cell back to world coordinates
//
// Why cost + 1?
//   Every step in a cardinal direction (no diagonals) has uniform cost.
//   This makes Dijkstra equivalent to BFS on an unweighted grid —
//   but the priority queue structure generalizes to weighted graphs.
//   If you later wanted diagonal movement (cost = sqrt(2) ≈ 1.414),
//   you'd just change the cost calculation.
//
// ────────────────────────────────────────────────────────────────────────────
nav_msgs::msg::Path Dijkstraplanning::plan(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    path.header.stamp = this->now();

    // ── Convert world coordinates to grid coordinates ───────────────────
    GraphNode start_node = world_to_grid(start);
    GraphNode goal_node = world_to_grid(goal);

    // ── Validate start and goal ─────────────────────────────────────────
    if (!is_on_map(start_node))
    {
        RCLCPP_ERROR(get_logger(), "Start position is outside the map!");
        return path;
    }
    if (!is_on_map(goal_node))
    {
        RCLCPP_ERROR(get_logger(), "Goal position is outside the map!");
        return path;
    }
    if (map_->data[grid_to_index(start_node)] != 0)
    {
        RCLCPP_ERROR(get_logger(), "Start position is inside an obstacle!");
        return path;
    }
    if (map_->data[grid_to_index(goal_node)] != 0)
    {
        RCLCPP_ERROR(get_logger(), "Goal position is inside an obstacle!");
        return path;
    }

    // ── 4-connected neighborhood ────────────────────────────────────────
    //
    // Right, Up, Left, Down — no diagonals.
    // This means paths on diagonals will look like staircases.
    // To add 8-connected movement, add: {1,1}, {1,-1}, {-1,1}, {-1,-1}
    // and change the cost for diagonals to 1.414 (sqrt(2)).
    //
    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1}
    };

    // ── Algorithm state ─────────────────────────────────────────────────
    //
    // all_nodes stores every GraphNode by value. When we create a new
    // node, we push_back it and record its index. Parent pointers are
    // indices into this vector, not raw pointers — this is safer because
    // vector reallocation would invalidate raw pointers.
    //
    std::vector<GraphNode> all_nodes;
    all_nodes.reserve(map_->info.width * map_->info.height / 4);

    // visited_set tracks which grid cells have been processed.
    // Using a set of flat indices (y * width + x) for O(1) lookup.
    std::set<int> visited_set;

    // The priority queue: min-heap by cost.
    // CompareGraphNode makes this a min-heap (lowest cost on top).
    std::priority_queue<
        std::pair<int, int>,            // (cost, index into all_nodes)
        std::vector<std::pair<int, int>>,
        std::greater<std::pair<int, int>> // min-heap: smallest cost first
    > pending;

    // ── Seed the queue with the start node ──────────────────────────────
    start_node.cost = 0;
    start_node.parent_idx = -1; // no parent — this is the root
    all_nodes.push_back(start_node);
    pending.push({0, 0}); // (cost=0, index=0)

    int goal_idx = -1; // will be set when we find the goal

    // ── Main loop ───────────────────────────────────────────────────────
    //
    // This is the heart of Dijkstra. Each iteration:
    //   1. Pop the cheapest node
    //   2. Check if it's the goal
    //   3. Expand its neighbors
    //
    // The algorithm terminates when:
    //   - We pop the goal node (path found), OR
    //   - The queue is empty (goal unreachable)
    //
    while (!pending.empty())
    {
        // Pop the cell with the lowest cost
        auto [current_cost, current_idx] = pending.top();
        pending.pop();

        const GraphNode& current = all_nodes[current_idx];
        int current_flat = grid_to_index(current);

        // Skip if already visited.
        // This can happen because we might add the same cell to the
        // queue multiple times (via different paths). The first time
        // we pop it is guaranteed to be the cheapest, so subsequent
        // pops are redundant.
        if (visited_set.count(current_flat))
        {
            continue;
        }
        visited_set.insert(current_flat);

        // ── Visualization: mark this cell as visited ────────────────────
        // Value 10 shows as a distinct color in RViz2.
        // Comment out these two lines in production — publishing after
        // every cell is extremely slow.
        visited_map_.data[current_flat] = 10;
        map_pub_->publish(visited_map_);

        // ── Goal check ──────────────────────────────────────────────────
        // If the cell we just popped IS the goal, we're done.
        // Because priority_queue always gives us the cheapest cell,
        // the first time we reach the goal is via the shortest path.
        if (current.x == goal_node.x && current.y == goal_node.y)
        {
            goal_idx = current_idx;
            break;
        }

        // ── Expand neighbors ────────────────────────────────────────────
        for (const auto& [dx, dy] : directions)
        {
            GraphNode neighbor;
            neighbor.x = current.x + dx;
            neighbor.y = current.y + dy;

            // Is the neighbor inside the map boundaries?
            if (!is_on_map(neighbor))
                continue;

            int neighbor_flat = grid_to_index(neighbor);

            // Has this cell already been visited?
            if (visited_set.count(neighbor_flat))
                continue;

            // Is this cell free space?
            // map_->data[] values: 0 = free, 100 = occupied, -1 = unknown
            // We only expand into cells with value 0 (free).
            if (map_->data[neighbor_flat] != 0)
                continue;

            // ── Add neighbor to the queue ───────────────────────────────
            neighbor.cost = current.cost + 1;
            neighbor.parent_idx = current_idx;

            int new_idx = static_cast<int>(all_nodes.size());
            all_nodes.push_back(neighbor);
            pending.push({neighbor.cost, new_idx});
        }
    }

    // ── Path reconstruction ─────────────────────────────────────────────
    //
    // If goal_idx is still -1, we never reached the goal — no path exists.
    //
    // Otherwise, start at the goal node and follow parent_idx pointers
    // back to the start. This builds the path in reverse order
    // (goal -> start), so we reverse it at the end.
    //
    // Each grid cell is converted back to world coordinates (meters)
    // using grid_to_world().
    //
    if (goal_idx == -1)
    {
        RCLCPP_WARN(get_logger(), "Goal is unreachable!");
        return path; // empty path
    }

    // Walk backwards from goal to start via parent pointers
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    int trace_idx = goal_idx;

    while (trace_idx != -1)
    {
        const GraphNode& node = all_nodes[trace_idx];

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_->header.frame_id;
        pose_stamped.header.stamp = this->now();
        pose_stamped.pose = grid_to_world(node);

        waypoints.push_back(pose_stamped);
        trace_idx = node.parent_idx;
    }

    // Reverse: the path was built goal -> start, we need start -> goal
    std::reverse(waypoints.begin(), waypoints.end());
    path.poses = waypoints;

    RCLCPP_INFO(get_logger(),
        "Path cost: %d steps, %zu waypoints",
        all_nodes[goal_idx].cost,
        path.poses.size());

    return path;
}


// ── Coordinate Conversion Helpers ───────────────────────────────────────────

// World (meters) -> Grid (cells)
//
// The map's origin is the world-coordinate position of the bottom-left pixel.
// To convert: subtract origin, divide by resolution, truncate to int.
//
// Example:
//   origin = (-5.0, -5.0), resolution = 0.05 m/pixel
//   world position (2.35, -1.07) ->
//   grid_x = (2.35 - (-5.0)) / 0.05 = 147
//   grid_y = (-1.07 - (-5.0)) / 0.05 = 78
//
GraphNode Dijkstraplanning::world_to_grid(
    const geometry_msgs::msg::Pose& pose) const
{
    GraphNode node;
    node.x = static_cast<int>(
        (pose.position.x - map_->info.origin.position.x)
        / map_->info.resolution);
    node.y = static_cast<int>(
        (pose.position.y - map_->info.origin.position.y)
        / map_->info.resolution);
    return node;
}


// Grid (cells) -> World (meters)
//
// The reverse of world_to_grid. Multiply by resolution, add origin.
// We add resolution/2 to center the pose in the middle of the cell
// rather than at its corner — without this, the path would follow
// cell edges instead of cell centers.
//
geometry_msgs::msg::Pose Dijkstraplanning::grid_to_world(
    const GraphNode& node) const
{
    geometry_msgs::msg::Pose pose;
    pose.position.x =
        map_->info.origin.position.x
        + (node.x + 0.5) * map_->info.resolution;
    pose.position.y =
        map_->info.origin.position.y
        + (node.y + 0.5) * map_->info.resolution;

    // Orientation defaults to identity quaternion (0,0,0,1)
    // which means "facing forward." The path planner doesn't
    // compute orientations — the controller handles that.
    pose.orientation.w = 1.0;

    return pose;
}


// Grid (x,y) -> Flat array index
//
// The OccupancyGrid::data is a 1D array stored in row-major order.
// Cell (x, y) maps to index y * width + x.
//
// For a 200-wide map:
//   cell (3, 7) -> index 7 * 200 + 3 = 1403
//   cell (0, 0) -> index 0 (top-left / origin corner)
//
int Dijkstraplanning::grid_to_index(const GraphNode& node) const
{
    return node.y * static_cast<int>(map_->info.width) + node.x;
}


// Bounds check: is this cell inside the map?
//
bool Dijkstraplanning::is_on_map(const GraphNode& node) const
{
    return node.x >= 0
        && node.x < static_cast<int>(map_->info.width)
        && node.y >= 0
        && node.y < static_cast<int>(map_->info.height);
}


// ── Main ────────────────────────────────────────────────────────────────────
//
// Standard ROS 2 main function:
//   1. Initialize rclcpp
//   2. Create the node
//   3. Spin (process callbacks forever)
//   4. Shutdown on Ctrl+C
//
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dijkstraplanning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}