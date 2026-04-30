#ifndef BALLBOT_PLANNING__DIJKSTRA_PLANNER_HPP_
#define BALLBOT_PLANNING__DIJKSTRA_PLANNER_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

namespace ballbot_planning // FIX 7: was bumperbot_planning
{

// ── Graph Node ───────────────────────────────────────────────────────────────
// FIX 8: Unified to use parent_idx (int) everywhere.
// The cpp was using shared_ptr<GraphNode> prev which conflicts with the hpp.
// parent_idx is faster (no heap allocation) and avoids dangling pointers.
struct GraphNode {
  int x = 0;
  int y = 0;
  int cost = 0;        // g(n) — steps from start + costmap cost
  int parent_idx = -1; // index into all_nodes, -1 = start node

  GraphNode() = default;
  GraphNode(int x, int y, int cost = 0, int parent_idx = -1)
      : x(x), y(y), cost(cost), parent_idx(parent_idx) {}

  bool operator==(const GraphNode &o) const { return x == o.x && y == o.y; }
};

// Min-heap comparator — lowest cost on top
struct CompareGraphNode {
  bool operator()(const GraphNode &a, const GraphNode &b) const {
    return a.cost > b.cost;
  }
};

// Hash for unordered_set — O(1) visited check instead of O(n) std::find
// FIX 5: replaces the slow std::vector + std::find pattern
struct GraphNodeHash {
  std::size_t operator()(const GraphNode &n) const {
    return std::hash<int>()(n.x) ^ (std::hash<int>()(n.y) << 16);
  }
};

// ── Dijkstra Nav2 Plugin ──────────────────────────────────────────────────
// FIX 1: Class name unified to DijkstraPlanner everywhere (was Dijkstraplanning
// in hpp)
class DijkstraPlanner : public nav2_core::GlobalPlanner {
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() override = default;

  // nav2_core::GlobalPlanner interface
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // FIX 3: Only declared here, implemented in cpp — no empty inline bodies
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal,
                                 std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D *costmap_{nullptr};
  std::string global_frame_;
  std::string name_;

  // FIX 2: smooth_client_ was used in cpp but missing from hpp
  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr
      smooth_client_;

  // Helpers
  GraphNode worldToGrid(const geometry_msgs::msg::Pose &pose) const;
  geometry_msgs::msg::Pose gridToWorld(const GraphNode &node) const;
  bool poseOnMap(const GraphNode &node) const;
};

} // namespace ballbot_planning

#endif // BALLBOT_PLANNING__DIJKSTRA_PLANNER_HPP_