#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException

from queue import PriorityQueue


class Graph_Node:
    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost          # g(n) — accumulated cost from start
        self.heuristic = 0        # h(n) — estimated cost to goal
        self.prev = prev

    # Priority queue orders by f(n) = g(n) + h(n)
    # This is the only behavioural difference from Dijkstra,
    # where the queue is ordered by g(n) alone.
    def __lt__(self, other: 'Graph_Node'):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other: 'Graph_Node'):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other: 'Graph_Node'):
        return Graph_Node(self.x + other[0], self.y + other[1])


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        map_qos = QoSProfile(depth=10)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.pose_callback, map_qos)

        # IMPORTANT: publishes on the SAME topics as the Dijkstra planner so
        # downstream consumers (controller, RViz config) need no changes.
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", qos_profile=map_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/map", qos_profile=map_qos)

        self.map_ = None
        self.visited_map = OccupancyGrid()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("A* planner initialized — waiting for map")

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map.header.frame_id = map_msg.header.frame_id
        self.visited_map.info = map_msg.info
        self.visited_map.data = [-1] * (map_msg.info.width * map_msg.info.height)

    def pose_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().info("Waiting for map")
            return

        self.visited_map.data = [-1] * (self.map_.info.width * self.map_.info.height)

        try:
            map_to_base_transform = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_link", rclpy.time.Time())
        except LookupException:
            self.get_logger().info("Waiting for transform")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_transform.transform.translation.x
        map_to_base_pose.position.y = map_to_base_transform.transform.translation.y
        map_to_base_pose.orientation = map_to_base_transform.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose)

        if path.poses:
            self.get_logger().info("Publishing path")
            self.path_pub.publish(path)
        else:
            self.get_logger().info("No path found")

    # ── Heuristic ────────────────────────────────────────────────────────────
    # Manhattan distance is admissible (never over-estimates) for a
    # 4-connected grid with unit step cost, so A* still returns the
    # optimal path while expanding far fewer cells than Dijkstra.
    def manhattan_heuristic(self, node: Graph_Node, goal: Graph_Node) -> int:
        return abs(node.x - goal.x) + abs(node.y - goal.y)

    def plan(self, start, goal):
        exploration_directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        pending_nodes = PriorityQueue()
        visited_nodes = set()

        start_node = self.world_grid(start)
        goal_node = self.world_grid(goal)

        start_node.heuristic = self.manhattan_heuristic(start_node, goal_node)
        pending_nodes.put(start_node)

        active_node = start_node
        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            if active_node == goal_node:
                break

            for dirx, diry in exploration_directions:
                new_node: Graph_Node = active_node + (dirx, diry)

                if new_node not in visited_nodes and self.pose_on_map(new_node) and \
                        self.map_.data[self.pose_to_call(new_node)] == 0:
                    new_node.cost = active_node.cost + 1
                    new_node.heuristic = self.manhattan_heuristic(new_node, goal_node)
                    new_node.prev = active_node
                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)

            self.visited_map.data[self.pose_to_call(active_node)] = 10
            self.map_pub.publish(self.visited_map)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id

        while active_node and active_node.prev and rclpy.ok():
            last_pose: Pose = self.grid_to_world(active_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            active_node = active_node.prev

        path.poses.reverse()
        return path

    def grid_to_world(self, node: Graph_Node) -> Pose:
        pose = Pose()
        pose.position.x = self.map_.info.origin.position.x + node.x * self.map_.info.resolution
        pose.position.y = self.map_.info.origin.position.y + node.y * self.map_.info.resolution
        return pose

    def world_grid(self, pose: Pose) -> Graph_Node:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return Graph_Node(grid_x, grid_y)

    def pose_on_map(self, node: Graph_Node):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def pose_to_call(self, node: Graph_Node):
        return node.y * self.map_.info.width + node.x


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
