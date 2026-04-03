import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Mynode(Node):
    def __init__(self):
        self.counter = 0
        super().__init__('smartphone')
        self.subcriber = self.create_subscription(String, 'robot_news', self.callback_robotnews, 10)
        self.get_logger().info('Smartphone has been started')
    def callback_robotnews(self, msg : String):
        self.get_logger().info('I heard %s and counter is %d' %(msg.data, self.counter))
        self.counter += 1



def main():
    rclpy.init()
    node = Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

