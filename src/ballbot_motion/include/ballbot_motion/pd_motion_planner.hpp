#ifndef BALLBOT_MOTION__PD_MOTION_PLANNER_HPP_
#define BALLBOT_MOTION__PD_MOTION_PLANNER_HPP_

// ============================================================================
// pd_motion_planner.hpp — PD Controller for Ballbot
//
// PD = Proportional + Derivative control.
//
// P term: corrects based on current error
//   output_p = Kp * error
//
// D term: corrects based on how fast error is changing (dampens overshoot)
//   output_d = Kd * (error - prev_error) / dt
//
// Total: output = Kp * error + Kd * d(error)/dt
//
// For your BALLBOT:
//   Linear error  → controls vx (forward/back)
//   Angular error → controls omega (rotation)
//   Lateral error → controls vy (sideways) ← BALLBOT ADDITION
//     Lateral control lets the robot slide sideways to reduce cross-track
//     error without rotating — the holonomic advantage.
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

class PDMotionPlanner : public nav2_core::Controller
{
public:
  PDMotionPlanner()  = default;
  ~PDMotionPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // Get the next waypoint the robot should aim toward
  geometry_msgs::msg::PoseStamped getNextPose(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  bool transformPlan(const std::string & frame);

  // ── Member variables ─────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PDMotionPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  // Publishes next target pose for RViz visualization
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>> next_pose_pub_;

  // ── PD Gains (tunable via nav2_params.yaml) ──────────────────────────
  double kp_;   // proportional gain — higher = more aggressive correction
  double kd_;   // derivative gain   — higher = more damping, less overshoot

  // ── Velocity limits ───────────────────────────────────────────────────
  double max_linear_velocity_;   // max vx (m/s)
  double max_angular_velocity_;  // max omega (rad/s)
  double max_lateral_velocity_;  // max vy (m/s) — BALLBOT ADDITION

  // How far along the path to look for the next waypoint
  double step_size_;

  // Speed limit from Nav2 (applied near obstacles)
  double speed_limit_{1.0};
  bool   speed_limit_is_percentage_{false};

  // ── PD state (stored between control cycles) ──────────────────────────
  // Needed for derivative term: d(error)/dt = (error - prev_error) / dt
  rclcpp::Time last_cycle_time_;
  double prev_angular_error_{0.0};
  double prev_linear_error_{0.0};
  double prev_lateral_error_{0.0};  // BALLBOT ADDITION

  nav_msgs::msg::Path global_plan_;
};

}  // namespace ballbot_motion

#endif  // BALLBOT_MOTION__PD_MOTION_PLANNER_HPP_
