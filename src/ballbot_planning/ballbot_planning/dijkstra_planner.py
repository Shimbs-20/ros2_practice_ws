#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener, LookupException

from queue import PriorityQueue


class Graph_Node:
    def __init__(self, x, y, cost = 0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev

    def __lt__(self, other: 'Graph_Node'): # for priority queue ot is an override method for less than
        return self.cost < other.cost
    def __eq__(self, other: 'Graph_Node'):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other: 'Graph_Node'):
        return Graph_Node(self.x + other[0], self.y + other[1])



class Dijkstrplanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
        map_qos = QoSProfile(depth=10)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid,"/map" ,self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseStamped,"/goal_pose" ,self.pose_callback, map_qos)
        self.path_pub = self.create_publisher(Path,"/dijkstra/path" , qos_profile= map_qos)
        self.map_pub = self.create_publisher(OccupancyGrid,"/dijkstra/map" , qos_profile= map_qos)

        self.map_ = None
        self.visited_map = OccupancyGrid()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #The planner needs the robot's current position in the map frame to use as the start point.
        #It gets this by looking up the transform from map → base_link:

    def map_callback(self,map_msg : OccupancyGrid): #in header frame of map it calls the map callback
        self.map_ = map_msg
        self.visited_map.header.frame_id = map_msg.header.frame_id
        self.visited_map.info = map_msg.info
        self.visited_map.data = [-1] * (map_msg.info.width * map_msg.info.height)

    def pose_callback(self,pose : PoseStamped):
        if self.map_ is None:
            self.get_logger().info("Waiting for map")
            return

        self.visited_map.data = [-1] * (self.map_.info.width * self.map_.info.height)

        try:
            map_to_base_transform = self.tf_buffer.lookup_transform(self.map_.header.frame_id, "base_link", rclpy.time.Time())
            #rclpy.time.Time() means "give me the latest available transform.
            # " The result is a TransformStamped containing translation (x, y, z) and rotation (quaternion).
            # The code extracts x and y as the robot's position on the map.

        except LookupException:
            self.get_logger().info("Waiting for transform")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_transform.transform.translation.x
        map_to_base_pose.position.y = map_to_base_transform.transform.translation.y
        map_to_base_pose.orientation = map_to_base_transform.transform.rotation

        Path = self.plan(map_to_base_pose, pose.pose)

        if Path.poses:
            self.get_logger().info("Publishing path")
            self.path_pub.publish(Path)
        else:
            self.get_logger().info("No path found")


    def plan(self, start, goal):
        exploration_directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        pending_nodes = PriorityQueue()
        visited_nodes = set()
        start_node = self.world_grid(start)
        pending_nodes.put(start_node)

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()
            #Pull the cheapest unvisited node from the queue. On the first iteration, this is the start node (cost 0).
            # On subsequent iterations, it's whatever node has the lowest accumulated cost.

            if active_node == self.world_grid(goal):
                break
            for dirx, diry in exploration_directions:
                new_node : Graph_Node= active_node + (dirx, diry)
                #Try each of the 4 neighbors.
                # The __add__ method creates a new Graph_Node at the neighboring cell.

                if new_node not in visited_nodes and self.pose_on_map(new_node) and self.map_.data[self.pose_to_call(new_node)] == 0 :
                    #Three checks before adding a neighbor to the queue: has it already been visited (skip if yes), is it within the map boundaries,
                    # and is the cell free space (value 0 in the occupancy grid, where 0 = free, 100 = occupied, -1 = unknown).
                    new_node.cost = active_node.cost + 1
                    new_node.prev = active_node
                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)
                    #The neighbor's cost is the current node's cost plus 1 (uniform cost for all steps).
                    # Set prev to point back to the current node — this is how we'll reconstruct the path.
                    # Add to the queue (to be processed later) and mark as visited.

            self.visited_map.data[self.pose_to_call(active_node)] = 10
            self.map_pub.publish(self.visited_map)
            #Visualization — mark the current cell as "visited" in the visualization map (value 10 shows as a light gray in RViz2) and publish it.
            # This lets you watch the flood-fill expansion in real time.
            # Note that this slows the algorithm down significantly because it publishes after every single cell.
            # In production you'd remove this or publish every N iterations.

        path = Path()
        path.header.frame_id = self.map_.header.frame_id

        while active_node and active_node.prev and rclpy.ok():
            last_pose : Pose = self.grid_to_world(active_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            active_node = active_node.prev
            #Starting from the goal node, follow the prev pointers back to the start. Each node's grid position is converted back to
            # world coordinates and added to the path.
            # The path is built goal-to-start, so it gets reversed at the end.

        path.poses.reverse()
        return path

    def grid_to_world(self, node : Graph_Node) -> Pose:
        pose = Pose()
        pose.position.x = self.map_.info.origin.position.x + node.x * self.map_.info.resolution
        pose.position.y = self.map_.info.origin.position.y + node.y * self.map_.info.resolution
        return pose



    def world_grid(self,pose : Pose) -> Graph_Node:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return Graph_Node(grid_x, grid_y)



    def pose_on_map(self, node :Graph_Node):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def pose_to_call(self, node : Graph_Node):
        return node.y * self.map_.info.width + node.x #converts node to index of array for map






def main():
    rclpy.init()
    node = Dijkstrplanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
