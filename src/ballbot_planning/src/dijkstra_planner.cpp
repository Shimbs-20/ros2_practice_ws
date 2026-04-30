// ============================================================================
// dijkstra_planner.cpp
//
// This file implements Dijkstra's algorithm as a Nav2 GlobalPlanner PLUGIN.
//
// What is a Nav2 plugin?
//   Instead of running as a standalone node, this code is compiled into a
//   shared library (.so file). Nav2's planner_server loads it at runtime
//   and calls its functions when a path is needed.
//
//   The planner_server owns the lifecycle — it calls:
//     configure()   → once at startup
//     activate()    → when Nav2 becomes active
//     createPlan()  → every time a new goal arrives
//     deactivate()  → when Nav2 shuts down
//     cleanup()     → final teardown
//
// Why Dijkstra?
//   Dijkstra explores cells in order of their accumulated cost from the start.
//   It always finds the SHORTEST path (minimum cost) but it explores in all
//   directions equally — no guidance toward the goal.
//   A* improves on this by adding a heuristic to bias expansion toward goal.
// ============================================================================

#include "ballbot_planning/dijkstra_planner.hpp"

#include <algorithm>   // std::reverse
#include <chrono>      // std::chrono::seconds
#include <queue>       // std::priority_queue
#include <unordered_set>
#include <vector>

#include "pluginlib/class_list_macros.hpp"  // PLUGINLIB_EXPORT_CLASS macro

namespace ballbot_planning
{

// ============================================================================
// configure()
//
// Called ONCE when the planner_server starts up.
// This is where you store references to Nav2's infrastructure:
//   - parent node (for logging, clock, parameters)
//   - TF buffer (for coordinate transforms)
//   - costmap (the map the algorithm will search)
//
// Parameters given by Nav2:
//   parent      → weak pointer to the planner_server lifecycle node
//   name        → the plugin name from yaml e.g. "DijkstraPlugin"
//   tf          → TF2 buffer already set up by Nav2
//   costmap_ros → the global costmap (map + inflation + obstacles)
// ============================================================================
void DijkstraPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Lock the weak_ptr to get a shared_ptr we can use for logging etc.
  // weak_ptr is used so the plugin doesn't keep the node alive if it dies
  node_ = parent.lock();
  name_ = name;
  tf_   = tf;

  // getCostmap() returns the underlying Costmap2D object.
  // This gives us direct access to cell costs without going through ROS topics.
  // The costmap is continuously updated by the obstacle layer and inflation layer.
  costmap_ = costmap_ros->getCostmap();

  // The frame the costmap is published in — typically "map"
  // All planned path poses must be in this frame
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Create the action client for optional path smoothing.
  // The smoother_server runs as a separate Nav2 node.
  // It receives our raw blocky grid path and returns a smooth curve.
  // This is optional — if the smoother isn't running, we skip smoothing.
  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(
      node_, "smooth_path");

  RCLCPP_INFO(node_->get_logger(),
      "DijkstraPlanner configured in frame: %s", global_frame_.c_str());
}

// ============================================================================
// cleanup() / activate() / deactivate()
//
// These are Nav2 lifecycle hooks — called when Nav2 transitions between states.
// For a simple planner plugin, we just log the transition.
// More complex plugins might allocate/free GPU memory here, for example.
// ============================================================================
void DijkstraPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(),
      "Cleaning up plugin %s of type DijkstraPlanner", name_.c_str());
}

void DijkstraPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(),
      "Activating plugin %s of type DijkstraPlanner", name_.c_str());

  // Check if the path smoother action server is available.
  // We wait up to 3 seconds. If it's not there, smoothing is skipped —
  // not a fatal error, just a warning.
  if (!smooth_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_WARN(node_->get_logger(),
        "smooth_path action server not available — path smoothing disabled");
  }
}

void DijkstraPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(),
      "Deactivating plugin %s of type DijkstraPlanner", name_.c_str());
}

// ============================================================================
// createPlan()
//
// This is the MAIN function — called every time Nav2 needs a path.
// It receives the robot's current pose and the goal pose, both as
// PoseStamped messages in the global frame (map).
//
// It must return a nav_msgs::msg::Path — a list of PoseStamped waypoints
// from start to goal that the controller (MPPI) will follow.
//
// cancel_checker: a function Nav2 gives us to check if the user cancelled.
//   We call it periodically during long planning runs.
//   If it returns true, we abort and return an empty path.
// ============================================================================
nav_msgs::msg::Path DijkstraPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp    = node_->now();

  // ── Step 1: Define exploration directions ─────────────────────────────
  // We explore 4-connected neighbors: right, left, up, down.
  // 8-connected (adding diagonals) would give smoother paths
  // but is harder to implement correctly with Dijkstra.
  const std::vector<std::pair<int, int>> directions = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}
  };

  // ── Step 2: Convert world poses to grid cells ─────────────────────────
  // The costmap is a 2D grid of cells. Each cell has a cost value:
  //   0         = free space (robot can move here)
  //   1-252     = inflated obstacle (robot should avoid but can go here)
  //   253       = inscribed obstacle (robot footprint would touch wall)
  //   254       = lethal obstacle (wall/obstacle cell)
  //   255 (-1)  = unknown space
  //
  // We convert real-world (x,y) metres to integer (col, row) grid indices.
  GraphNode start_node = worldToGrid(start.pose);
  GraphNode goal_node  = worldToGrid(goal.pose);

  // ── Step 3: Validate start and goal ───────────────────────────────────
  // Planning from inside a wall or outside the map makes no sense.
  // Nav2 normally validates this too, but we check defensively.
  if (!poseOnMap(start_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Start is outside the map");
    return path;  // return empty path → Nav2 will trigger recovery
  }
  if (!poseOnMap(goal_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is outside the map");
    return path;
  }
  // getCost() returns the costmap value at (x, y).
  // 253 = inscribed radius — robot body would clip the wall.
  if (costmap_->getCost(start_node.x, start_node.y) >= 253) {
    RCLCPP_ERROR(node_->get_logger(), "Start is inside an obstacle");
    return path;
  }
  if (costmap_->getCost(goal_node.x, goal_node.y) >= 253) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is inside an obstacle");
    return path;
  }

  // ── Step 4: Data structures ───────────────────────────────────────────

  // visited: tracks cells we have fully processed.
  // Using unordered_set with a hash function gives O(1) lookup.
  // Alternative: std::vector + std::find = O(n) lookup = very slow on large maps.
  std::unordered_set<GraphNode, GraphNodeHash> visited;

  // all_nodes: flat array storing every node we create during the search.
  // Each node stores its grid position (x,y), cost, and parent_idx.
  // parent_idx points back into this array — how we reconstruct the path.
  // This avoids raw pointer or shared_ptr chains (safer, cache-friendly).
  std::vector<GraphNode> all_nodes;
  all_nodes.reserve(1024);  // pre-allocate to avoid repeated reallocation

  // pq: the priority queue (min-heap).
  // Dijkstra always processes the LOWEST cost node next.
  // Stores pairs: (cost, index into all_nodes)
  // std::greater makes it a min-heap (default is max-heap)
  std::priority_queue<
      std::pair<int, int>,
      std::vector<std::pair<int, int>>,
      std::greater<std::pair<int, int>>
  > pq;

  // ── Step 5: Initialize with start node ───────────────────────────────
  start_node.cost       = 0;    // start costs nothing to reach
  start_node.parent_idx = -1;   // no parent — this is the beginning
  all_nodes.push_back(start_node);
  pq.push({0, 0});  // push (cost=0, index=0)

  int goal_idx = -1;  // will hold index of goal node in all_nodes when found

  // ── Step 6: Main Dijkstra loop ────────────────────────────────────────
  while (!pq.empty() && rclcpp::ok()) {

    // Check if Nav2 cancelled the planning request (user sent new goal etc.)
    if (cancel_checker && cancel_checker()) {
      RCLCPP_INFO(node_->get_logger(), "Planning cancelled");
      return path;
    }

    // Pop the node with the LOWEST cost from the priority queue.
    // This is the core of Dijkstra — always expand the cheapest node.
    // cur_cost = accumulated cost to reach this node
    // cur_idx  = its index in all_nodes
    auto [cur_cost, cur_idx] = pq.top();
    pq.pop();

    GraphNode current = all_nodes[cur_idx];

    // Skip if already visited (can be in queue multiple times with different costs)
    if (visited.count(current)) { continue; }
    visited.insert(current);

    // ── Goal check ────────────────────────────────────────────────────
    // If we just expanded the goal cell, we are done.
    // Dijkstra guarantees this is the LOWEST cost path to the goal.
    if (current == goal_node) {
      goal_idx = cur_idx;
      break;
    }

    // ── Explore 4 neighbors ───────────────────────────────────────────
    for (const auto & [dx, dy] : directions) {
      GraphNode nb(current.x + dx, current.y + dy);

      // Skip if out of map bounds or already visited
      if (!poseOnMap(nb) || visited.count(nb)) { continue; }

      // Read the costmap value for this cell.
      // unsigned char range: 0-255
      unsigned char cell_cost = costmap_->getCost(nb.x, nb.y);

      // Skip lethal obstacles (walls) and inscribed radius cells.
      // 253 = inscribed obstacle (robot footprint touches wall)
      // 254 = lethal obstacle (actual wall cell)
      if (cell_cost >= 253) { continue; }

      // Calculate new cost:
      //   +1 for each step (uniform grid distance)
      //   +cell_cost so paths through high-cost inflated areas are penalized
      //   This naturally routes the robot through the CENTER of corridors
      //   away from walls, even without explicit wall avoidance logic.
      nb.cost       = current.cost + 1 + static_cast<int>(cell_cost);
      nb.parent_idx = cur_idx;  // remember how we got here

      int nb_idx = static_cast<int>(all_nodes.size());
      all_nodes.push_back(nb);
      pq.push({nb.cost, nb_idx});
    }
  }

  // ── Step 7: Check if goal was reached ─────────────────────────────────
  if (goal_idx == -1) {
    RCLCPP_WARN(node_->get_logger(), "Dijkstra: goal unreachable");
    return path;  // empty path → Nav2 triggers recovery behavior
  }

  // ── Step 8: Reconstruct path by following parent_idx chain ────────────
  // Starting from the goal, follow parent_idx back to start.
  // This builds the path in REVERSE (goal → start).
  // We reverse it at the end to get start → goal order.
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  int idx = goal_idx;
  while (idx != -1) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = global_frame_;
    ps.header.stamp    = node_->now();
    // Convert grid cell back to world coordinates (metres)
    ps.pose = gridToWorld(all_nodes[idx]);
    waypoints.push_back(ps);
    idx = all_nodes[idx].parent_idx;  // move to parent
  }
  std::reverse(waypoints.begin(), waypoints.end());
  path.poses = waypoints;

  RCLCPP_INFO(node_->get_logger(),
      "Dijkstra: path found — %zu waypoints, %zu cells expanded",
      path.poses.size(), visited.size());

  // ── Step 9: Optional path smoothing ───────────────────────────────────
  // The raw Dijkstra path is blocky (grid-aligned, 90-degree turns).
  // The Nav2 smoother_server can smooth it into curves the MPPI controller
  // can follow more naturally.
  //
  // We send the raw path to the /smooth_path action server.
  // If it's not available (smoother_server not launched), we skip this.
  if (smooth_client_->action_server_is_ready()) {
    nav2_msgs::action::SmoothPath::Goal smooth_goal;
    smooth_goal.path                   = path;
    smooth_goal.check_for_collisions   = false;  // trust our obstacle check above
    smooth_goal.smoother_id            = "simple_smoother";
    smooth_goal.max_smoothing_duration = rclcpp::Duration(10, 0);  // 10s timeout

    // Send goal and wait up to 3 seconds for acceptance
    auto future = smooth_client_->async_send_goal(smooth_goal);
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto handle = future.get();
      if (handle) {
        // Wait up to 3 seconds for the smoothed path result
        auto result_future = smooth_client_->async_get_result(handle);
        if (result_future.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready) {
          auto result = result_future.get();
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            path = result.result->path;  // replace blocky path with smooth one
            RCLCPP_INFO(node_->get_logger(), "Path smoothed successfully");
          }
        }
      }
    }
  }

  return path;
}

// ============================================================================
// worldToGrid()
//
// Converts a real-world pose (metres) to a grid cell (integer indices).
//
// The costmap has an origin (its bottom-left corner in world coordinates).
// To find which cell a world point belongs to:
//   1. Subtract the origin to get position relative to map corner
//   2. Divide by resolution (metres per cell) to get cell index
//
// Example:
//   origin = (-12.5, -12.5), resolution = 0.05m
//   world point (0.0, 0.0) → cell (250, 250)  [centre of map]
//   world point (2.0, 1.5) → cell (290, 280)
// ============================================================================
GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose) const
{
  return GraphNode(
      static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution()),
      static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution())
  );
}

// ============================================================================
// gridToWorld()
//
// Converts a grid cell back to real-world coordinates (metres).
// The +0.5 * resolution centres the pose in the MIDDLE of the cell,
// not at its corner. This prevents the path from zigzagging along cell edges.
// ============================================================================
geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = node.x * costmap_->getResolution()
                    + costmap_->getOriginX()
                    + 0.5 * costmap_->getResolution();  // centre of cell
  pose.position.y = node.y * costmap_->getResolution()
                    + costmap_->getOriginY()
                    + 0.5 * costmap_->getResolution();
  pose.orientation.w = 1.0;  // facing forward (yaw = 0)
  return pose;
}

// ============================================================================
// poseOnMap()
//
// Returns true if the given grid cell is within the costmap boundaries.
// Must be called BEFORE getCost() to avoid out-of-bounds access.
// ============================================================================
bool DijkstraPlanner::poseOnMap(const GraphNode & node) const
{
  return node.x >= 0
      && node.x < static_cast<int>(costmap_->getSizeInCellsX())
      && node.y >= 0
      && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

}  // namespace ballbot_planning

// ============================================================================
// PLUGINLIB_EXPORT_CLASS macro
//
// This is REQUIRED for Nav2 to discover and load this plugin.
// It generates the C symbols that pluginlib uses to instantiate the class.
//
// First argument:  the fully qualified class name (namespace::ClassName)
// Second argument: the base class interface it implements
//
// This must match EXACTLY what is in plugins.xml:
//   type="ballbot_planning::DijkstraPlanner"
//   base_class_type="nav2_core::GlobalPlanner"
// ============================================================================
PLUGINLIB_EXPORT_CLASS(ballbot_planning::DijkstraPlanner, nav2_core::GlobalPlanner)