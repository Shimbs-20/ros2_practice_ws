import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import Hardwarestatus

class Hardwarestatuspublisher(Node):
    def __init__(self):
        self.counter = 0
        super().__init__('Hradware_status')
        self.hw_status = self.create_publisher(Hardwarestatus, 'hardware_status', 10)
        self.timer = self.create_timer(1.0, self.callback_robotnews)
        self.get_logger().info('Hardware_status has been started')

    def callback_robotnews(self):
        msg = Hardwarestatus()
        msg.tempreture = 30.8
        msg.motors_ready = True
        msg.debug_message = "Debug message"
        self.hw_status.publish(msg)




def main():
    rclpy.init()
    node = Hardwarestatuspublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

