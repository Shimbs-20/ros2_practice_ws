import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
class Mynode(Node):
    def __init__(self):
        super().__init__("robot_station")
        self.counter = 0
        self.robot_name = "Shimbs"
        self.publisher1 = self.create_publisher(Int32, "number_count", 10)
        self.subscribe = self.create_subscription(
            Int32, "data" , self.callback_news, 10)
        self.get_logger().info("Robot news station INT started")

    def callback_news(self, msg : Int32):
        self.counter += msg.data
        new_msg = Int32()
        new_msg.data = self.counter
        self.publisher1.publish(new_msg)


def main():
    rclpy.init()
    node = Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
