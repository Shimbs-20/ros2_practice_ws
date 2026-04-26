// ============================================================================
// a_star_planner.cpp — Implementation
// ============================================================================
//
// Structurally identical to dijkstra_planner.cpp. The only behavioural change
// is in plan(): the priority queue is keyed on f(n) = g(n) + h(n) rather than
// g(n) alone, where h(n) is the Manhattan distance to the goal.
//
// Because the heuristic biases expansion toward the goal, A* visits a small
// fraction of the cells Dijkstra would, giving a large speed-up on open maps.
//
// ============================================================================

#include "/home/youssef-shemela/ros2_ws/src/ballbot_planning/include/ballbot_planning/a_star_planner.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <set>
#include <vector>


// ── Constructor ─────────────────────────────────────────────────────────────
AStarPlanning::AStarPlanning()
    : Node("a_star_planner")
{
    rclcpp::QoS map_qos(10);
    map_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        map_qos,
        std::bind(&AStarPlanning::map_callback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        map_qos,
        std::bind(&AStarPlanning::pose_callback, this, std::placeholders::_1));

    // Same output topics as the Dijkstra node — downstream consumers do not
    // need to know which algorithm produced the path.
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/a_star/path", map_qos);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/a_star/map", map_qos);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "A* planner initialized — waiting for map");
}


// ── map_callback ────────────────────────────────────────────────────────────
void AStarPlanning::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = msg;

    visited_map_.header.frame_id = msg->header.frame_id;
    visited_map_.info = msg->info;
    visited_map_.data.assign(
        msg->info.width * msg->info.height,
        -1);

    RCLCPP_INFO(get_logger(),
        "Map received: %d x %d, resolution: %.3f m/pixel",
        msg->info.width, msg->info.height, msg->info.resolution);
}


// ── pose_callback ───────────────────────────────────────────────────────────
void AStarPlanning::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
    if (!map_)
    {
        RCLCPP_WARN(get_logger(), "No map received yet — cannot plan");
        return;
    }

    std::fill(
        visited_map_.data.begin(),
        visited_map_.data.end(),
        -1);

    geometry_msgs::msg::TransformStamped map_to_base;
    try
    {
        map_to_base = tf_buffer_->lookupTransform(
            map_->header.frame_id,
            "base_link",
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(get_logger(),
            "Cannot find transform map -> base_link: %s", ex.what());
        return;
    }

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = map_to_base.transform.translation.x;
    start_pose.position.y = map_to_base.transform.translation.y;
    start_pose.orientation = map_to_base.transform.rotation;

    RCLCPP_INFO(get_logger(),
        "Planning from (%.2f, %.2f) to (%.2f, %.2f)",
        start_pose.position.x, start_pose.position.y,
        goal_msg->pose.position.x, goal_msg->pose.position.y);

    auto path = plan(start_pose, goal_msg->pose);

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


// ── manhattan_heuristic ─────────────────────────────────────────────────────
int AStarPlanning::manhattan_heuristic(
    const AStarGraphNode& a, const AStarGraphNode& b) const
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}


// ── plan() — A* Algorithm ───────────────────────────────────────────────────
//
// Identical loop structure to Dijkstra. Differences are marked with "A*:".
//
// ────────────────────────────────────────────────────────────────────────────
nav_msgs::msg::Path AStarPlanning::plan(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    path.header.stamp = this->now();

    AStarGraphNode start_node = world_to_grid(start);
    AStarGraphNode goal_node = world_to_grid(goal);

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

    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1}
    };

    std::vector<AStarGraphNode> all_nodes;
    all_nodes.reserve(map_->info.width * map_->info.height / 4);

    std::set<int> visited_set;

    // A*: priority queue is keyed on f = g + h (first element of the pair).
    std::priority_queue<
        std::pair<int, int>,            // (f_cost, index into all_nodes)
        std::vector<std::pair<int, int>>,
        std::greater<std::pair<int, int>>
    > pending;

    start_node.cost = 0;
    start_node.heuristic = manhattan_heuristic(start_node, goal_node);
    start_node.parent_idx = -1;
    all_nodes.push_back(start_node);
    pending.push({start_node.f(), 0});

    int goal_idx = -1;

    while (!pending.empty())
    {
        auto [current_f, current_idx] = pending.top();
        pending.pop();

        const AStarGraphNode& current = all_nodes[current_idx];
        int current_flat = grid_to_index(current);

        if (visited_set.count(current_flat))
        {
            continue;
        }
        visited_set.insert(current_flat);

        visited_map_.data[current_flat] = 10;
        map_pub_->publish(visited_map_);

        if (current.x == goal_node.x && current.y == goal_node.y)
        {
            goal_idx = current_idx;
            break;
        }

        for (const auto& [dx, dy] : directions)
        {
            AStarGraphNode neighbor;
            neighbor.x = current.x + dx;
            neighbor.y = current.y + dy;

            if (!is_on_map(neighbor))
                continue;

            int neighbor_flat = grid_to_index(neighbor);

            if (visited_set.count(neighbor_flat))
                continue;

            if (map_->data[neighbor_flat] != 0)
                continue;

            neighbor.cost = current.cost + 1;
            // A*: compute the heuristic for the new neighbor.
            neighbor.heuristic = manhattan_heuristic(neighbor, goal_node);
            neighbor.parent_idx = current_idx;

            int new_idx = static_cast<int>(all_nodes.size());
            all_nodes.push_back(neighbor);
            // A*: push f(n) = g + h into the queue, not g alone.
            pending.push({neighbor.f(), new_idx});
        }
    }

    if (goal_idx == -1)
    {
        RCLCPP_WARN(get_logger(), "Goal is unreachable!");
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    int trace_idx = goal_idx;

    while (trace_idx != -1)
    {
        const AStarGraphNode& node = all_nodes[trace_idx];

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_->header.frame_id;
        pose_stamped.header.stamp = this->now();
        pose_stamped.pose = grid_to_world(node);

        waypoints.push_back(pose_stamped);
        trace_idx = node.parent_idx;
    }

    std::reverse(waypoints.begin(), waypoints.end());
    path.poses = waypoints;

    RCLCPP_INFO(get_logger(),
        "Path cost: %d steps, %zu waypoints, %zu cells expanded",
        all_nodes[goal_idx].cost,
        path.poses.size(),
        visited_set.size());

    return path;
}


// ── Coordinate Conversion Helpers ───────────────────────────────────────────

AStarGraphNode AStarPlanning::world_to_grid(
    const geometry_msgs::msg::Pose& pose) const
{
    AStarGraphNode node;
    node.x = static_cast<int>(
        (pose.position.x - map_->info.origin.position.x)
        / map_->info.resolution);
    node.y = static_cast<int>(
        (pose.position.y - map_->info.origin.position.y)
        / map_->info.resolution);
    return node;
}


geometry_msgs::msg::Pose AStarPlanning::grid_to_world(
    const AStarGraphNode& node) const
{
    geometry_msgs::msg::Pose pose;
    pose.position.x =
        map_->info.origin.position.x
        + (node.x + 0.5) * map_->info.resolution;
    pose.position.y =
        map_->info.origin.position.y
        + (node.y + 0.5) * map_->info.resolution;
    pose.orientation.w = 1.0;
    return pose;
}


int AStarPlanning::grid_to_index(const AStarGraphNode& node) const
{
    return node.y * static_cast<int>(map_->info.width) + node.x;
}


bool AStarPlanning::is_on_map(const AStarGraphNode& node) const
{
    return node.x >= 0
        && node.x < static_cast<int>(map_->info.width)
        && node.y >= 0
        && node.y < static_cast<int>(map_->info.height);
}


// ── Main ────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlanning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
