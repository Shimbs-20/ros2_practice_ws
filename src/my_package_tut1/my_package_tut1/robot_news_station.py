import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
class Mynode(Node):
    def __init__(self):
        super().__init__("robot_station")
        self.robot_name = "Shimbs"
        self.publisher1 = self.create_publisher(String,"robot_news",10)
        self.create_timer(0.5, self.publishe_news)
        self.get_logger().info("Robot news station firststarted")


    def publishe_news(self):
        msg = String()
        msg.data = "Hello Robot" + self.robot_name + "!"
        self.publisher1.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
