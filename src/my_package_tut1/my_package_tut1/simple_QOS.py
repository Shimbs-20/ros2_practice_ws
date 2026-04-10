import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from example_interfaces.msg import String
class simple_Qospublisher_node(Node):
    def __init__(self):
        super().__init__("robot_station")


        self.qos_profile = QoSProfile(depth=10)


        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")


        self.reliability = self.get_parameter("reliability").get_parameter_value().string_value
        self.durability = self.get_parameter("durability").get_parameter_value().string_value
        
        if self.reliability == "best_effort":
            self.qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("reliability is best effort")
        elif self.reliability == "reliable":
            self.qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("reliability is RELIABLE")
        elif self.reliability == "system_default":
            self.qos_profile.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("reliability is SYSTEM_DEFAULT")
        else:
            self.get_logger().error("reliability is not valid")
            return

        if self.durability == "transient_local":
            self.qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("durability is TRANSIENT_LOCAL")
        elif self.durability == "transient":
            self.qos_profile.durability = QoSDurabilityPolicy.TRANSIENT
            self.get_logger().info("durability is TRANSIENT")
        elif self.durability == "system_default":
            self.qos_profile.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("durability is SYSTEM_DEFAULT")
        else:
            self.get_logger().error("reliability is not valid")
            return

        # self.declare_parameter("number", 200)
        # self.declare_parameter("timer_period", 0.5)
        # self.data = self.get_parameter("number").value
        # self.timer = self.get_parameter("timer_period").value

        self.robot_name = "Shimbs"
        self.frequency = 1.0
        self.publisher1 = self.create_publisher(String,"QOS_chatter",self.qos_profile)
        self.create_timer(self.frequency, self.publishe_news)
        self.get_logger().info("Robot news station INT started")



    def publishe_news(self):
        msg = String()
        msg.data = self.robot_name
        self.publisher1.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    simple_Qospublisher = simple_Qospublisher_node()
    rclpy.spin(simple_Qospublisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
