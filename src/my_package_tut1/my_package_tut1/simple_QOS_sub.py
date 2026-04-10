import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from example_interfaces.msg import String
class SimpleQOSSubcriber(Node):
    def __init__(self):
        super().__init__("Simple Quality of Service subscriber")
        self.counter = 0
        self.robot_name = "Shimbs"

        self.qos_profile_sub = QoSProfile(depth=10)


        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")


        self.reliability = self.get_parameter("reliability").get_parameter_value().string_value
        self.durability = self.get_parameter("durability").get_parameter_value().string_value

        if self.reliability == "best_effort":
            self.qos_profile_sub_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("reliability is best effort")
        elif self.reliability == "reliable":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("reliability is RELIABLE")
        elif self.reliability == "system_default":
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("reliability is SYSTEM_DEFAULT")
        else:
            self.get_logger().error("reliability is not valid")
            return

        if self.durability == "transient_local":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("durability is TRANSIENT_LOCAL")
        elif self.durability == "transient":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.TRANSIENT
            self.get_logger().info("durability is TRANSIENT")
        elif self.durability == "system_default":
            self.qos_profile_sub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("durability is SYSTEM_DEFAULT")
        else:
            self.get_logger().error("reliability is not valid")
            return


        self.subscribe = self.create_subscription(
            String, "data" , self.callback_news, self.qos_profile_sub)
        self.get_logger().info("Robot news station INT started")

    def callback_news(self, msg : String):
        # self.counter += msg.data
        new_msg = msg
        # new_msg.data = self.counter
        self.get_logger().info("QOS INT is %d" %(new_msg.data))


def main():
    rclpy.init()
    node = SimpleQOSSubcriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
