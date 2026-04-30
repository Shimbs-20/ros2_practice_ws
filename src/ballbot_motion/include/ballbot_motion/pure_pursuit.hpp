#ifndef BALLBOT_MOTION__PURE_PURSUIT_HPP_
#define BALLBOT_MOTION__PURE_PURSUIT_HPP_

// ============================================================================
// pure_pursuit.hpp — Pure Pursuit Controller for Ballbot
//
// Pure Pursuit is a path tracking algorithm. It works by:
//   1. Finding a "carrot" point on the path at look_ahead_distance ahead
//   2. Computing the curvature of the arc needed to reach the carrot
//   3. Converting curvature to angular velocity command
//
// For your BALLBOT (holonomic/omnidirectional):
//   Standard pure pursuit only controls vx and omega (differential drive).
//   The ballbot version ADDS vy (lateral velocity) computation.
//   This allows the robot to slide sideways toward the path instead of
//   rotating to face it first — the true holonomic advantage.
// ============================================================================

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace ballbot_motion   // FIX: was bumperbot_motion
{

class PurePursuit : public nav2_core::Controller
{
public:
  PurePursuit()  = default;
  ~PurePursuit() override = default;

  // ── Nav2 Controller Plugin Interface ─────────────────────────────────
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  // Called at controller_frequency (e.g. 20Hz) to compute cmd_vel
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // Called when a new global path arrives from the planner
  void setPlan(const nav_msgs::msg::Path & path) override;

  // Called when Nav2 wants to limit speed (e.g. near obstacles)
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // Find the carrot point: the path point at look_ahead_distance from robot
  geometry_msgs::msg::PoseStamped getCarrotPose(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  // Convert path from map frame to robot frame
  bool transformPlan(const std::string & frame);

  // Compute arc curvature to carrot point: k = 2y / (x² + y²)
  double getCurvature(const geometry_msgs::msg::Pose & carrot_pose);

  // BALLBOT ADDITION: compute lateral velocity toward path
  // For holonomic robots — slides robot sideways to correct cross-track error
  double getLateralVelocity(const geometry_msgs::msg::Pose & carrot_pose);

  // ── Member variables ─────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuit")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  // Publishes the carrot point for visualization in RViz
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>> carrot_pub_;

  // ── Parameters (set via nav2_params.yaml) ────────────────────────────
  double look_ahead_distance_;   // how far ahead to place the carrot (metres)
  double max_linear_velocity_;   // max vx (m/s)
  double max_angular_velocity_;  // max omega (rad/s)
  double max_lateral_velocity_;  // max vy (m/s) — BALLBOT ONLY, 0 for diff drive

  // Speed limit applied by Nav2 (e.g. near obstacles)
  double speed_limit_{1.0};
  bool   speed_limit_is_percentage_{false};

  nav_msgs::msg::Path global_plan_;
};

}  // namespace ballbot_motion

#endif  // BALLBOT_MOTION__PURE_PURSUIT_HPP_
