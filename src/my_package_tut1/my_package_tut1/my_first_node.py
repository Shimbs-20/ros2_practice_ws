import rclpy
from rclpy.node import Node


class Mynode(Node):
    def __init__(self):
        super().__init__("python_node")
        self.counter = 0
        self.get_logger().info("Hello ROS2")
        self.create_timer(1.0, self.timer_call_back)

    def timer_call_back(self):
        self.get_logger().info(f"Hello Shimbs {self.counter}")
        self.counter += 1


def main(args=None):
    # Initialize the ROS2 Python library with the provided arguments
    rclpy.init(args=args)

    # Create a ROS2 node named 'my_first_node'
    node = Mynode()
    # Get the logger object for the node and log the message 'Hello ROS2'
    #. The get_logger() method returns the logger object associated with the node,
    # and the info() method logs the message at the INFO level.

    # The spin() method runs the ROS2 event loop, which processes
    rclpy.spin(node)
    #because node is spinning in the background the function callback will be called every 1 second because of the timer
    # Shut down the ROS2 library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
