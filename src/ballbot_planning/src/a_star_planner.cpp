// ============================================================================
// a_star_planner.cpp
//
// A* (A-Star) global planner — Nav2 plugin.
//
// A* vs Dijkstra:
//   Dijkstra expands cells in order of g(n) = actual cost from start.
//   It explores in ALL directions equally — like a wave radiating outward.
//
//   A* expands cells in order of f(n) = g(n) + h(n), where:
//     g(n) = actual cost from start (same as Dijkstra)
//     h(n) = heuristic: ESTIMATED cost from n to goal
//
//   The heuristic GUIDES the search toward the goal, so A* expands
//   far fewer cells on average.
//
//   Requirement: h(n) must NEVER overestimate the true cost to goal.
//   This is called "admissibility" and guarantees A* still finds optimal path.
//   Manhattan distance (|dx| + |dy|) is admissible on a 4-connected grid
//   because you can never reach the goal in fewer steps than |dx| + |dy|.
//
// Structure:
//   Identical to dijkstra_planner.cpp EXCEPT:
//   1. GraphNode has an extra heuristic field
//   2. Priority queue is keyed on f(n) = g + h, not just g
//   3. manhattanDistance() is computed for each neighbor
//   4. PathAngle critic weight should be 0 in MPPI config (see nav2_params.yaml)
// ============================================================================

#include "ballbot_planning/a_star_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>       // std::abs
#include <queue>
#include <unordered_set>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace ballbot_planning
{

// ============================================================================
// configure() — identical to Dijkstra
//
// The only difference is the class name in the log messages.
// Everything else is exactly the same — same costmap, same TF, same smoother.
// This shows that Nav2's plugin interface cleanly separates "what algorithm"
// from "how it gets its data" — the algorithm is just createPlan().
// ============================================================================
void AStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_         = parent.lock();
  name_         = name;
  tf_           = tf;
  costmap_      = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(
      node_, "smooth_path");

  RCLCPP_INFO(node_->get_logger(),
      "AStarPlanner configured in frame: %s", global_frame_.c_str());
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(),
      "Cleaning up plugin %s of type AStarPlanner", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(),
      "Activating plugin %s of type AStarPlanner", name_.c_str());
  if (!smooth_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_WARN(node_->get_logger(),
        "smooth_path action server not available — path smoothing disabled");
  }
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(),
      "Deactivating plugin %s of type AStarPlanner", name_.c_str());
}

// ============================================================================
// createPlan() — THE KEY DIFFERENCE FROM DIJKSTRA IS HERE
//
// Steps 1-4 (directions, conversion, validation, data structures) are
// identical to Dijkstra.
//
// Step 5: start node gets a heuristic value immediately.
// Step 6: priority queue is keyed on f(n) = g + h, NOT just g.
// Step 7: each neighbor gets a heuristic computed before being pushed.
//
// Everything else (path reconstruction, smoothing) is identical.
// ============================================================================
nav_msgs::msg::Path AStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp    = node_->now();

  const std::vector<std::pair<int, int>> directions = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}
  };

  AStarNode start_node = worldToGrid(start.pose);
  AStarNode goal_node  = worldToGrid(goal.pose);

  // Validate — same as Dijkstra
  if (!poseOnMap(start_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Start is outside the map");
    return path;
  }
  if (!poseOnMap(goal_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is outside the map");
    return path;
  }
  if (costmap_->getCost(start_node.x, start_node.y) >= 253) {
    RCLCPP_ERROR(node_->get_logger(), "Start is inside an obstacle");
    return path;
  }
  if (costmap_->getCost(goal_node.x, goal_node.y) >= 253) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is inside an obstacle");
    return path;
  }

  // O(1) visited lookup — same as Dijkstra
  std::unordered_set<AStarNode, AStarNodeHash> visited;

  // Flat node storage with parent_idx for path reconstruction — same as Dijkstra
  std::vector<AStarNode> all_nodes;
  all_nodes.reserve(1024);

  // ── A* KEY DIFFERENCE 1: Priority queue keyed on f(n) = g + h ────────
  // Dijkstra: pq.push({g, index})
  // A*:       pq.push({f, index})  where f = g + h
  //
  // This means A* always expands the node that looks most PROMISING
  // (lowest estimated total cost through that node).
  // Dijkstra expands nodes purely by distance from start.
  std::priority_queue<
      std::pair<int, int>,
      std::vector<std::pair<int, int>>,
      std::greater<std::pair<int, int>>
  > pq;  // (f_cost, index into all_nodes)

  // ── A* KEY DIFFERENCE 2: Start node gets a heuristic ─────────────────
  // Dijkstra: start_node.cost = 0, no heuristic
  // A*:       start_node.cost = 0, start_node.heuristic = distance to goal
  //
  // At the start, h = manhattan distance to goal.
  // As we expand outward, nodes CLOSER to the goal get lower h values
  // and are processed first — pulling the search toward the goal.
  start_node.cost       = 0;
  start_node.heuristic  = manhattanDistance(start_node, goal_node);
  start_node.parent_idx = -1;
  all_nodes.push_back(start_node);
  pq.push({start_node.f(), 0});  // f() = cost + heuristic = 0 + h

  int goal_idx = -1;

  // ── Main A* loop ──────────────────────────────────────────────────────
  while (!pq.empty() && rclcpp::ok()) {

    if (cancel_checker && cancel_checker()) {
      RCLCPP_INFO(node_->get_logger(), "A* planning cancelled");
      return path;
    }

    // Pop node with LOWEST f(n) = g + h
    // This is the node that appears "most promising" toward the goal
    auto [cur_f, cur_idx] = pq.top();
    pq.pop();

    AStarNode current = all_nodes[cur_idx];

    if (visited.count(current)) { continue; }
    visited.insert(current);

    // Goal check — same as Dijkstra
    // Because h is admissible, when we first expand the goal cell,
    // we are guaranteed to have found the optimal path.
    if (current == goal_node) {
      goal_idx = cur_idx;
      break;
    }

    // ── Explore neighbors ─────────────────────────────────────────────
    for (const auto & [dx, dy] : directions) {
      AStarNode nb(current.x + dx, current.y + dy);

      if (!poseOnMap(nb) || visited.count(nb)) { continue; }

      unsigned char cell_cost = costmap_->getCost(nb.x, nb.y);
      if (cell_cost >= 253) { continue; }

      // g(n): actual cost to reach this neighbor
      // Same formula as Dijkstra: steps + costmap cost
      nb.cost = current.cost + 1 + static_cast<int>(cell_cost);

      // ── A* KEY DIFFERENCE 3: Compute heuristic for each neighbor ─────
      // Dijkstra: no heuristic — just push with g cost
      // A*:       compute Manhattan distance to goal for each neighbor
      //           h = |nb.x - goal.x| + |nb.y - goal.y|
      //
      // Neighbors CLOSER to the goal get LOWER h values.
      // In the priority queue these get processed FIRST.
      // This is what makes A* faster — it preferentially expands
      // toward the goal instead of expanding in all directions.
      nb.heuristic  = manhattanDistance(nb, goal_node);
      nb.parent_idx = cur_idx;

      int nb_idx = static_cast<int>(all_nodes.size());
      all_nodes.push_back(nb);

      // ── Push with f(n) = g + h ────────────────────────────────────
      // Dijkstra: pq.push({nb.cost, nb_idx})         ← keyed on g only
      // A*:       pq.push({nb.f(), nb_idx})           ← keyed on g + h
      pq.push({nb.f(), nb_idx});
    }
  }

  if (goal_idx == -1) {
    RCLCPP_WARN(node_->get_logger(), "A*: goal unreachable");
    return path;
  }

  // ── Path reconstruction — identical to Dijkstra ───────────────────────
  // Follow parent_idx chain from goal back to start, then reverse.
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  int idx = goal_idx;
  while (idx != -1) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = global_frame_;
    ps.header.stamp    = node_->now();
    ps.pose = gridToWorld(all_nodes[idx]);
    waypoints.push_back(ps);
    idx = all_nodes[idx].parent_idx;
  }
  std::reverse(waypoints.begin(), waypoints.end());
  path.poses = waypoints;

  RCLCPP_INFO(node_->get_logger(),
      "A*: path found — %zu waypoints, %zu cells expanded",
      path.poses.size(), visited.size());

  // ── Optional smoothing — identical to Dijkstra ────────────────────────
  // Raw A* path is still grid-aligned (blocky).
  // Smoother converts it to a smooth curve for MPPI to follow.
  if (smooth_client_->action_server_is_ready()) {
    nav2_msgs::action::SmoothPath::Goal smooth_goal;
    smooth_goal.path                   = path;
    smooth_goal.check_for_collisions   = false;
    smooth_goal.smoother_id            = "simple_smoother";
    smooth_goal.max_smoothing_duration = rclcpp::Duration(10, 0);

    auto future = smooth_client_->async_send_goal(smooth_goal);
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto handle = future.get();
      if (handle) {
        auto result_future = smooth_client_->async_get_result(handle);
        if (result_future.wait_for(std::chrono::seconds(3)) ==
            std::future_status::ready) {
          auto result = result_future.get();
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            path = result.result->path;
            RCLCPP_INFO(node_->get_logger(), "Path smoothed successfully");
          }
        }
      }
    }
  }

  return path;
}

// ============================================================================
// manhattanDistance()
//
// h(n) = |n.x - goal.x| + |n.y - goal.y|
//
// This counts the minimum number of grid steps needed to reach the goal
// if there were NO obstacles. Since obstacles can only ADD to the path,
// this never OVERESTIMATES — making it admissible.
//
// Admissibility = h(n) ≤ true_cost(n, goal)  for all n
//
// Why Manhattan and not Euclidean?
//   Euclidean = sqrt(dx²+dy²) — gives the straight-line distance.
//   On a 4-connected grid you can't move diagonally, so:
//     Manhattan distance ≤ actual grid distance ✅ admissible
//     Euclidean distance ≤ Manhattan distance   ✅ also admissible but weaker
//   Manhattan is tighter (higher h values) → fewer cells expanded → faster.
// ============================================================================
int AStarPlanner::manhattanDistance(const AStarNode & a, const AStarNode & b) const
{
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// ============================================================================
// worldToGrid() / gridToWorld() / poseOnMap()
// Identical to Dijkstra — see comments there.
// ============================================================================
AStarNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose) const
{
  return AStarNode(
      static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution()),
      static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution())
  );
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const AStarNode & node) const
{
  geometry_msgs::msg::Pose pose;
  // +0.5 centres the world pose inside the cell (not at its corner)
  pose.position.x = node.x * costmap_->getResolution()
                    + costmap_->getOriginX()
                    + 0.5 * costmap_->getResolution();
  pose.position.y = node.y * costmap_->getResolution()
                    + costmap_->getOriginY()
                    + 0.5 * costmap_->getResolution();
  pose.orientation.w = 1.0;
  return pose;
}

bool AStarPlanner::poseOnMap(const AStarNode & node) const
{
  return node.x >= 0
      && node.x < static_cast<int>(costmap_->getSizeInCellsX())
      && node.y >= 0
      && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

}  // namespace ballbot_planning

// Register plugin with pluginlib — Nav2 finds this at runtime
PLUGINLIB_EXPORT_CLASS(ballbot_planning::AStarPlanner, nav2_core::GlobalPlanner)