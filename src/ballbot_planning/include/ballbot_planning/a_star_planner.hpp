#ifndef BALLBOT_PLANNING__A_STAR_PLANNER_HPP_
#define BALLBOT_PLANNING__A_STAR_PLANNER_HPP_

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

namespace ballbot_planning {

// ── Graph Node (A* version — adds heuristic field) ────────────────────────
struct AStarNode {
  int x = 0;
  int y = 0;
  int cost = 0;      // g(n) — actual cost from start
  int heuristic = 0; // h(n) — Manhattan distance to goal
  int parent_idx = -1;

  AStarNode() = default;
  AStarNode(int x, int y, int cost = 0, int heuristic = 0, int parent_idx = -1)
      : x(x), y(y), cost(cost), heuristic(heuristic), parent_idx(parent_idx) {}

  // f(n) = g(n) + h(n) — A* priority
  int f() const { return cost + heuristic; }

  bool operator==(const AStarNode &o) const { return x == o.x && y == o.y; }
};

// Min-heap on f(n) = g + h
struct CompareAStarNode {
  bool operator()(const AStarNode &a, const AStarNode &b) const {
    return a.f() > b.f();
  }
};

// Hash for O(1) visited lookup
struct AStarNodeHash {
  std::size_t operator()(const AStarNode &n) const {
    return std::hash<int>()(n.x) ^ (std::hash<int>()(n.y) << 16);
  }
};

// ── A* Nav2 Plugin ────────────────────────────────────────────────────────
class AStarPlanner : public nav2_core::GlobalPlanner {
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

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

  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr
      smooth_client_;

  // Manhattan distance heuristic — admissible for 4-connected grid
  int manhattanDistance(const AStarNode &a, const AStarNode &b) const;

  AStarNode worldToGrid(const geometry_msgs::msg::Pose &pose) const;
  geometry_msgs::msg::Pose gridToWorld(const AStarNode &node) const;
  bool poseOnMap(const AStarNode &node) const;
};

} // namespace ballbot_planning

#endif // BALLBOT_PLANNING__A_STAR_PLANNER_HPP_