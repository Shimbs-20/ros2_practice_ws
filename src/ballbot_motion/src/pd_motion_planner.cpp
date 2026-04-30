#include "ballbot_motion/pd_motion_planner.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ballbot_motion
{

// ============================================================================
// configure()
//
// PD controller parameters explained:
//
//   kp (proportional gain):
//     How hard to correct current error.
//     kp too low  → robot drifts, doesn't follow path tightly
//     kp too high → robot oscillates left-right
//
//   kd (derivative gain):
//     How hard to dampen rapidly changing error.
//     kd too low  → robot overshoots waypoints
//     kd too high → robot freezes (over-dampened)
//
//   step_size:
//     Which waypoint along the path to aim at.
//     step_size=1 → aim at next waypoint (aggressive)
//     step_size=5 → aim 5 waypoints ahead (smoother but less precise)
// ============================================================================
void PDMotionPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent;
  auto node    = node_.lock();
  costmap_ros_ = costmap_ros;
  tf_buffer_   = tf_buffer;
  plugin_name_ = name;
  logger_      = node->get_logger();
  clock_       = node->get_clock();

  // PD gains
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kd", rclcpp::ParameterValue(0.1));

  // Lookahead step along path
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".step_size", rclcpp::ParameterValue(1));

  // Velocity limits
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",  rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  // BALLBOT ADDITION: lateral velocity limit
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_velocity", rclcpp::ParameterValue(0.3));

  node->get_parameter(plugin_name_ + ".kp",                  kp_);
  node->get_parameter(plugin_name_ + ".kd",                  kd_);
  node->get_parameter(plugin_name_ + ".step_size",           step_size_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity",  max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".max_lateral_velocity", max_lateral_velocity_);

  // Initialize PD state — no previous error on startup
  last_cycle_time_    = clock_->now();
  prev_angular_error_ = 0.0;
  prev_linear_error_  = 0.0;
  prev_lateral_error_ = 0.0;

  // Publisher for next target pose visualization in RViz
  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "pd_controller/next_pose", 1);

  RCLCPP_INFO(logger_,
    "PDMotionPlanner configured: kp=%.2f, kd=%.2f, step=%.0f, "
    "vx_max=%.2f, omega_max=%.2f, vy_max=%.2f",
    kp_, kd_, step_size_, max_linear_velocity_, max_angular_velocity_, max_lateral_velocity_);
}

void PDMotionPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PDMotionPlanner");
  next_pose_pub_.reset();
}

void PDMotionPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating PDMotionPlanner");
  next_pose_pub_->on_activate();
}

void PDMotionPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PDMotionPlanner");
  next_pose_pub_->on_deactivate();
}

void PDMotionPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_, "New path received: %zu poses", path.poses.size());
  global_plan_ = path;

  // Reset PD state when new path arrives — old errors are no longer relevant
  prev_angular_error_ = 0.0;
  prev_linear_error_  = 0.0;
  prev_lateral_error_ = 0.0;
  last_cycle_time_    = clock_->now();
}

void PDMotionPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_               = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ============================================================================
// computeVelocityCommands()
//
// PD control loop for holonomic ballbot:
//
//   For ANGULAR control (heading to next waypoint):
//     error     = angle between robot heading and direction to target
//     P term    = kp * angular_error
//     D term    = kd * (angular_error - prev_angular_error) / dt
//     omega     = clamp(P + D, -max_omega, +max_omega)
//
//   For LINEAR control (distance to next waypoint):
//     error     = distance to target pose
//     P term    = kp * linear_error
//     D term    = kd * (linear_error - prev_linear_error) / dt
//     vx        = clamp(P + D, 0, max_vx)
//
//   For LATERAL control (BALLBOT ONLY — sideways correction):
//     error     = cross-track error (perpendicular distance to path)
//     P term    = kp * lateral_error
//     D term    = kd * (lateral_error - prev_lateral_error) / dt
//     vy        = clamp(P + D, -max_vy, +max_vy)
// ============================================================================
geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp    = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "Empty plan — cannot compute velocity");
    return cmd_vel;
  }

  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR(logger_, "Failed to transform plan");
    return cmd_vel;
  }

  // ── Get the next target pose ─────────────────────────────────────────
  auto next_pose = getNextPose(robot_pose);
  next_pose_pub_->publish(next_pose);

  // ── Compute dt for derivative term ───────────────────────────────────
  // dt = time since last control cycle
  // Used to convert error difference to error rate: d(error)/dt
  auto now = clock_->now();
  double dt = (now - last_cycle_time_).seconds();
  last_cycle_time_ = now;

  // Avoid division by zero or huge dt on first cycle
  if (dt <= 0.0 || dt > 1.0) { dt = 0.05; }

  // ── Angular PD control ───────────────────────────────────────────────
  // Compute direction from robot to next waypoint
  double dx    = next_pose.pose.position.x - robot_pose.pose.position.x;
  double dy    = next_pose.pose.position.y - robot_pose.pose.position.y;
  double target_yaw = std::atan2(dy, dx);

  // Get robot's current yaw from quaternion
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Angular error = difference between target direction and robot heading
  // Normalize to [-π, π]
  double angular_error = target_yaw - robot_yaw;
  while (angular_error >  M_PI) { angular_error -= 2.0 * M_PI; }
  while (angular_error < -M_PI) { angular_error += 2.0 * M_PI; }

  // PD terms for angular
  double angular_p = kp_ * angular_error;
  double angular_d = kd_ * (angular_error - prev_angular_error_) / dt;
  prev_angular_error_ = angular_error;

  cmd_vel.twist.angular.z = std::clamp(
    angular_p + angular_d,
    -max_angular_velocity_,
     max_angular_velocity_);

  // ── Linear PD control ────────────────────────────────────────────────
  double linear_error = std::hypot(dx, dy);

  double linear_p = kp_ * linear_error;
  double linear_d = kd_ * (linear_error - prev_linear_error_) / dt;
  prev_linear_error_ = linear_error;

  // Apply speed limit
  double max_vel = max_linear_velocity_;
  if (speed_limit_is_percentage_) {
    max_vel *= speed_limit_;
  } else if (speed_limit_ < max_vel) {
    max_vel = speed_limit_;
  }

  // vx is always positive (forward) — clamped to [0, max]
  cmd_vel.twist.linear.x = std::clamp(linear_p + linear_d, 0.0, max_vel);

  // ── BALLBOT ADDITION: Lateral PD control ─────────────────────────────
  // Cross-track error = perpendicular distance from robot to path direction
  // Positive → robot is to the right of path → slide left (positive vy)
  // Negative → robot is to the left of path  → slide right (negative vy)
  //
  // We compute it using the 2D cross product:
  //   path_direction = (dx, dy) normalized
  //   robot_to_target = (dx, dy)
  //   cross_track = robot position projected perpendicularly onto path
  //
  // For simplicity: use the y-component of (target - robot) in path frame
  // This is the lateral offset from the line connecting robot to waypoint
  if (max_lateral_velocity_ > 0.0) {
    // Rotate (dx, dy) into robot frame to get lateral component
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    // dy_robot = component of error perpendicular to robot heading
    double lateral_error = -sin_yaw * dx + cos_yaw * dy;

    double lateral_p = kp_ * lateral_error;
    double lateral_d = kd_ * (lateral_error - prev_lateral_error_) / dt;
    prev_lateral_error_ = lateral_error;

    cmd_vel.twist.linear.y = std::clamp(
      lateral_p + lateral_d,
      -max_lateral_velocity_,
       max_lateral_velocity_);
  }

  return cmd_vel;
}

// ============================================================================
// getNextPose()
//
// Returns the waypoint on the path that is step_size poses ahead of the
// closest point to the robot.
//
// Steps:
//   1. Find the closest path point to the robot's current position
//   2. Advance step_size poses further along the path
//   3. Return that pose as the target
// ============================================================================
geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.empty()) {
    return robot_pose;
  }

  // Find the index of the closest path point to the robot
  size_t closest_idx = 0;
  double min_dist    = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx   = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy   = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist    = dist;
      closest_idx = i;
    }
  }

  // Advance step_size ahead, clamped to the last pose
  size_t target_idx = std::min(
    closest_idx + static_cast<size_t>(step_size_),
    global_plan_.poses.size() - 1);

  return global_plan_.poses[target_idx];
}

// ============================================================================
// transformPlan() — identical to PurePursuit version
// Transforms all path poses into the target frame using TF2
// ============================================================================
bool PDMotionPlanner::transformPlan(const std::string & frame)
{
  if (global_plan_.header.frame_id == frame) { return true; }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger_, "Cannot transform plan: %s", ex.what());
    return false;
  }

  for (auto & pose : global_plan_.poses) {
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace ballbot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ballbot_motion::PDMotionPlanner, nav2_core::Controller)
