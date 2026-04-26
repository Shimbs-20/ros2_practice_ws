#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSDurabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue


class Graph_Node:
    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev

    def __lt__(self, other: 'Graph_Node'):
        return self.cost < other.cost

    def __eq__(self, other: 'Graph_Node'):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return Graph_Node(self.x + other[0], self.y + other[1])


class Dijkstrplanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')

        # Map uses TRANSIENT_LOCAL — map_server latches it
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # BUG FIX 1: /goal_pose from RViz2 "2D Goal Pose" uses VOLATILE.
        # Using TRANSIENT_LOCAL here = QoS incompatible = message silently
        # dropped = pose_callback never fires = no path ever computed.
        goal_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Publishers use VOLATILE so RViz receives live updates
        pub_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos)

        # BUG FIX 1: use goal_qos not map_qos
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.pose_callback, goal_qos)

        self.path_pub = self.create_publisher(Path, "/dijkstra/path", pub_qos)
        self.map_pub  = self.create_publisher(OccupancyGrid, "/dijkstra/map", pub_qos)

        self.map_ = None
        self.visited_map = OccupancyGrid()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map.header.frame_id = map_msg.header.frame_id
        self.visited_map.info = map_msg.info
        self.visited_map.data = [-1] * (map_msg.info.width * map_msg.info.height)
        self.get_logger().info(
            f"Map received: {map_msg.info.width}x{map_msg.info.height}")

    def pose_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().warn("Waiting for map")
            return

        self.visited_map.data = [-1] * (self.map_.info.width * self.map_.info.height)

        # BUG FIX 4: AMCL publishes map→odom→base_footprint, not base_link.
        # lookupTransform("map", "base_link") always raises LookupException.
        try:
            map_to_base = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time())
        except LookupException as e:
            self.get_logger().warn(f"Transform not available: {e}")
            return

        start_pose = Pose()
        start_pose.position.x = map_to_base.transform.translation.x
        start_pose.position.y = map_to_base.transform.translation.y
        start_pose.orientation = map_to_base.transform.rotation

        self.get_logger().info(
            f"Planning: ({start_pose.position.x:.2f},{start_pose.position.y:.2f})"
            f" → ({pose.pose.position.x:.2f},{pose.pose.position.y:.2f})")

        # BUG FIX 2: was "Path = self.plan(...)" which overwrote the
        # imported Path class, causing a TypeError when plan() later
        # tried to create "path = Path()". Renamed to "result".
        result = self.plan(start_pose, pose.pose)

        if result.poses:
            self.get_logger().info(f"Publishing path: {len(result.poses)} waypoints")
            self.path_pub.publish(result)
        else:
            self.get_logger().warn("No path found")

    def plan(self, start: Pose, goal: Pose) -> Path:
        exploration_directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        pending_nodes = PriorityQueue()
        visited_nodes = set()

        start_node = self.world_grid(start)
        goal_node  = self.world_grid(goal)

        pending_nodes.put(start_node)
        active_node = start_node
        cells_expanded = 0

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            if active_node == goal_node:
                break

            for dirx, diry in exploration_directions:
                new_node : Graph_Node = active_node + (dirx, diry)

                if (new_node not in visited_nodes
                        and self.pose_on_map(new_node)
                        and self.map_.data[self.pose_to_call(new_node)] < 99 and self.map_.data[self.pose_to_call(new_node)] >= 0):
                    new_node.cost = active_node.cost + 1
                    new_node.prev = active_node
                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)

            self.visited_map.data[self.pose_to_call(active_node)] = 10
            cells_expanded += 1

            # BUG FIX 3: was publishing visited map after EVERY cell.
            # On a 500x500 map = 250,000 OccupancyGrid publishes.
            # This completely blocks the executor so path_pub.publish()
            # never fires even though the path is computed correctly.
            # Now publish every 500 cells only.
            if cells_expanded % 500 == 0:
                self.map_pub.publish(self.visited_map)

        # Final visualization update
        self.map_pub.publish(self.visited_map)

        # Build path by following prev pointers from goal → start
        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        node = active_node
        while node is not None and node.prev is not None and rclpy.ok():
            ps = PoseStamped()
            ps.header.frame_id = self.map_.header.frame_id
            ps.header.stamp    = path.header.stamp
            ps.pose = self.grid_to_world(node)
            path.poses.append(ps)
            node = node.prev

        path.poses.reverse()
        return path

    def grid_to_world(self, node: Graph_Node) -> Pose:
        pose = Pose()
        # +0.5 centres the pose in the middle of the cell
        pose.position.x = (self.map_.info.origin.position.x
                           + (node.x + 0.5) * self.map_.info.resolution)
        pose.position.y = (self.map_.info.origin.position.y
                           + (node.y + 0.5) * self.map_.info.resolution)
        pose.orientation.w = 1.0
        return pose

    def world_grid(self, pose: Pose) -> Graph_Node:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x)
                     / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y)
                     / self.map_.info.resolution)
        return Graph_Node(grid_x, grid_y)

    def pose_on_map(self, node: Graph_Node) -> bool:

        return (0 <= node.x < self.map_.info.width
                and 0 <= node.y < self.map_.info.height)

    def pose_to_call(self, node: Graph_Node) -> int:
        return node.y * self.map_.info.width + node.x


def main():
    rclpy.init()
    node = Dijkstrplanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
