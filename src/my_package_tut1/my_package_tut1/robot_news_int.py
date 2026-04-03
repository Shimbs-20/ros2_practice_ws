import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
class Mynode(Node):
    def __init__(self):
        super().__init__("robot_station")
        self.declare_parameter("number", 200)
        self.declare_parameter("timer_period", 0.5)
        self.data = self.get_parameter("number").value
        self.timer = self.get_parameter("timer_period").value

        self.robot_name = "Shimbs"
        self.publisher1 = self.create_publisher(Int32,"data",10)
        self.create_timer(self.timer, self.publishe_news)
        self.get_logger().info("Robot news station INT started")



    def publishe_news(self):
        msg = Int32()
        msg.data = self.data
        self.publisher1.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
