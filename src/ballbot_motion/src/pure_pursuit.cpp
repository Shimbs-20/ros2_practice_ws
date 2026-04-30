#include "ballbot_motion/pure_pursuit.hpp"  // FIX: was bumperbot_motion

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
// Called once at startup by Nav2's controller_server.
// Sets up parameters, publishers, and stores references.
//
// Parameters declared here will appear in your nav2_params.yaml under:
//   controller_server:
//     ros__parameters:
//       FollowPath:          ← plugin name
//         look_ahead_distance: 0.5
//         max_linear_velocity: 0.3
//         ...
// ============================================================================
void PurePursuit::configure(
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

  // declare_parameter_if_not_declared: safe to call even if param already set
  // This allows the user to override from the yaml file
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",  rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  // BALLBOT ADDITION: lateral velocity parameter
  // Set to 0.0 for differential drive robots — they cannot move sideways
  // Set to > 0 for your ballbot to enable true holonomic control
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_velocity", rclcpp::ParameterValue(0.3));

  node->get_parameter(plugin_name_ + ".look_ahead_distance",  look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity",  max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".max_lateral_velocity", max_lateral_velocity_);

  // Publish the carrot point so we can visualize it in RViz
  // Shows where the robot is aiming at each control cycle
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "pure_pursuit/carrot", 1);

  RCLCPP_INFO(logger_, "PurePursuit configured: look_ahead=%.2f, vx_max=%.2f, "
    "omega_max=%.2f, vy_max=%.2f",
    look_ahead_distance_, max_linear_velocity_,
    max_angular_velocity_, max_lateral_velocity_);
}

// ============================================================================
// cleanup() / activate() / deactivate()
//
// Lifecycle hooks. The publisher must be activated/deactivated with the node.
// In Nav2 all publishers are LifecyclePublishers — they only publish when
// the node is in the ACTIVE state. This prevents publishing during startup.
// ============================================================================
void PurePursuit::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PurePursuit");
  carrot_pub_.reset();  // destroys the publisher and frees resources
}

void PurePursuit::activate()
{
  RCLCPP_INFO(logger_, "Activating PurePursuit");
  carrot_pub_->on_activate();  // enables publishing
}

void PurePursuit::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PurePursuit");
  carrot_pub_->on_deactivate();  // disables publishing without destroying
}

// ============================================================================
// setPlan()
//
// Called by Nav2 whenever a new global path arrives from the planner server.
// We just store it — the actual tracking happens in computeVelocityCommands().
// ============================================================================
void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_, "New path received: %zu poses in frame '%s'",
    path.poses.size(), path.header.frame_id.c_str());
  global_plan_ = path;
}

// ============================================================================
// setSpeedLimit()
//
// Called by Nav2 when it wants to limit speed (e.g. near obstacles, narrow
// passages). We store the limit and apply it in computeVelocityCommands().
//
// percentage: if true, speed_limit is 0.0-1.0 (fraction of max)
//             if false, speed_limit is in m/s directly
// ============================================================================
void PurePursuit::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_              = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

// ============================================================================
// computeVelocityCommands()
//
// The MAIN function — called at controller_frequency (e.g. 20Hz).
//
// Input:  robot's current pose, current velocity, goal checker
// Output: TwistStamped with (vx, vy, omega) to send to the robot
//
// Pure Pursuit algorithm:
//   1. Transform path into robot's current frame
//   2. Find the "carrot" point at look_ahead_distance ahead
//   3. Compute curvature of arc from robot to carrot: k = 2y / (x² + y²)
//   4. omega = k * max_angular_velocity
//   5. vx = max_linear_velocity (constant forward speed)
//   6. vy = lateral correction (BALLBOT ADDITION)
// ============================================================================
geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & /*velocity*/,  // unused — pure pursuit ignores current vel
  nav2_core::GoalChecker * /*goal_checker*/)        // unused — Nav2 handles goal checking
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp    = clock_->now();

  // Safety checks
  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "Empty plan — cannot compute velocity");
    return cmd_vel;  // zero velocity = robot stops
  }

  // Transform plan into the robot's current frame (e.g. base_footprint)
  // Required because the path comes from the planner in the map frame
  // but we need distances relative to the robot
  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR(logger_, "Failed to transform plan to frame: %s",
      robot_pose.header.frame_id.c_str());
    return cmd_vel;
  }

  // ── Step 1: Find the carrot point ──────────────────────────────────
  // The carrot is the path point at look_ahead_distance from the robot.
  // Like a carrot on a stick — always just ahead, pulling the robot forward.
  auto carrot_pose = getCarrotPose(robot_pose);
  carrot_pub_->publish(carrot_pose);  // visualize in RViz

  // ── Step 2: Transform carrot into robot frame ──────────────────────
  // We need the carrot position RELATIVE TO THE ROBOT (not world frame)
  // to compute the turning curvature.
  //
  // robot_tf:       robot pose in world frame
  // carrot_pose_tf: carrot pose in world frame
  // carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf
  //                      = carrot relative to robot
  tf2::Transform carrot_pose_tf, robot_tf, carrot_pose_robot_tf;
  tf2::fromMsg(robot_pose.pose,  robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
  carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf;
  tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);

  // ── Step 3: Compute curvature and velocity commands ────────────────
  double curvature = getCurvature(carrot_pose.pose);

  // Apply speed limit from Nav2 (obstacle proximity etc.)
  double linear_vel = max_linear_velocity_;
  if (speed_limit_is_percentage_) {
    linear_vel *= speed_limit_;
  } else if (speed_limit_ < max_linear_velocity_) {
    linear_vel = speed_limit_;
  }

  cmd_vel.twist.linear.x  = linear_vel;

  // omega = curvature × max_angular_velocity
  // curvature > 0: turn left, curvature < 0: turn right
  // Clamp to max_angular_velocity
  cmd_vel.twist.angular.z = std::clamp(
    curvature * max_angular_velocity_,
    -max_angular_velocity_,
     max_angular_velocity_);

  // ── BALLBOT ADDITION: Lateral velocity ────────────────────────────
  // Standard pure pursuit: vy = 0 (differential drive can't move sideways)
  // Ballbot pure pursuit: vy = lateral correction toward path
  //
  // The carrot's y coordinate in robot frame = lateral error to path.
  // Positive y = carrot is to the left → robot should slide left (positive vy)
  // Negative y = carrot is to the right → robot should slide right (negative vy)
  //
  // This allows the ballbot to SLIDE SIDEWAYS to correct cross-track error
  // without rotating — much smoother than differential drive.
  if (max_lateral_velocity_ > 0.0) {
    double lateral_vel = getLateralVelocity(carrot_pose.pose);
    cmd_vel.twist.linear.y = std::clamp(
      lateral_vel,
      -max_lateral_velocity_,
       max_lateral_velocity_);
  }

  return cmd_vel;
}

// ============================================================================
// getCarrotPose()
//
// Finds the "carrot" — the point on the path that is look_ahead_distance
// ahead of the robot.
//
// Algorithm:
//   Iterate path from END to START (reverse order).
//   The first point we find that is FURTHER than look_ahead_distance
//   is our carrot.
//
// Why reverse iteration?
//   We want the LAST point on the path that is still within look_ahead_distance.
//   If we iterated forward, we'd get the first close point, missing that
//   the robot may have passed it and the relevant point is further along.
//
// FIX: The original logic was subtly wrong — it kept setting carrot_pose
// to every point further than look_ahead_distance, ending with the first
// far point instead of the boundary point. Fixed with cleaner logic.
// ============================================================================
geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Default to the last point on path (goal itself)
  geometry_msgs::msg::PoseStamped carrot_pose = global_plan_.poses.back();

  for (auto it = global_plan_.poses.rbegin(); it != global_plan_.poses.rend(); ++it) {
    double dx       = it->pose.position.x - robot_pose.pose.position.x;
    double dy       = it->pose.position.y - robot_pose.pose.position.y;
    double distance = std::hypot(dx, dy);  // std::hypot is safer than sqrt(dx²+dy²)

    if (distance > look_ahead_distance_) {
      carrot_pose = *it;
    } else {
      // Found first point closer than look_ahead_distance — stop here
      break;
    }
  }
  return carrot_pose;
}

// ============================================================================
// getCurvature()
//
// Computes the curvature of the circular arc connecting robot to carrot.
//
// The carrot pose is in the ROBOT'S frame:
//   x = how far ahead the carrot is
//   y = how far left/right the carrot is
//
// Pure Pursuit geometric formula:
//   k = 2y / (x² + y²)
//
// Where (x, y) is the carrot position in robot frame.
// k > 0 → turn left
// k < 0 → turn right
// k = 0 → carrot is directly ahead (go straight)
//
// The 0.001 threshold avoids division by near-zero
// (robot is essentially on top of the carrot)
// ============================================================================
double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
  const double dist_sq =
    carrot_pose.position.x * carrot_pose.position.x +
    carrot_pose.position.y * carrot_pose.position.y;

  if (dist_sq > 0.001) {
    return 2.0 * carrot_pose.position.y / dist_sq;
  }
  return 0.0;  // carrot is too close — go straight
}

// ============================================================================
// getLateralVelocity() — BALLBOT ADDITION
//
// Computes a lateral (sideways) velocity for holonomic robots.
//
// The carrot's y position in robot frame is the lateral error:
//   y > 0 → carrot is to the left  → slide left  (positive vy)
//   y < 0 → carrot is to the right → slide right (negative vy)
//
// We use a proportional relationship: vy = k * y
// where k is chosen so that max_lateral_velocity maps to look_ahead_distance.
// ============================================================================
double PurePursuit::getLateralVelocity(const geometry_msgs::msg::Pose & carrot_pose)
{
  // Proportional lateral correction
  // Normalize by look_ahead_distance so the gain is distance-independent
  double lateral_error = carrot_pose.position.y;
  double vy = (lateral_error / look_ahead_distance_) * max_lateral_velocity_;
  return vy;
}

// ============================================================================
// transformPlan()
//
// Transforms all path poses from the map frame into the robot's current frame.
// This is required because getCurvature() needs the carrot position in the
// robot's local coordinate system.
//
// If the path is already in the target frame, returns immediately.
// Uses TF2 to look up the current transform at time=0 (latest available).
// ============================================================================
bool PurePursuit::transformPlan(const std::string & frame)
{
  // Nothing to do if already in the right frame
  if (global_plan_.header.frame_id == frame) {
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      frame,
      global_plan_.header.frame_id,
      tf2::TimePointZero);  // latest available transform
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger_, "Cannot transform plan from '%s' to '%s': %s",
      global_plan_.header.frame_id.c_str(), frame.c_str(), ex.what());
    return false;
  }

  // Apply transform to every pose in the plan
  for (auto & pose : global_plan_.poses) {
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace ballbot_motion

#include "pluginlib/class_list_macros.hpp"
// FIX: namespace changed from bumperbot_motion to ballbot_motion
PLUGINLIB_EXPORT_CLASS(ballbot_motion::PurePursuit, nav2_core::Controller)
