import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class turtle(Node):
    def __init__(self):
        super().__init__('turtlesim_kinematics')
        self.turtle1_pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.turtle1Posecallback,
            10)
        self.turtle2_pose_sub = self.create_subscription(
            Pose,
            "/turtle2/pose",
            self.turtle1Posecallback2,
            10)

        self.last_pose1= Pose()
        self.last_pose2= Pose()

    def turtle1Posecallback(self, msg):
        self.last_pose1 = msg

    def turtle1Posecallback2(self, msg):
        self.last_pose2 = msg
        Tx = self.last_pose2.x - self.last_pose1.x
        Ty = self.last_pose2.y - self.last_pose1.y

        self.get_logger().info(f'Tx: {Tx} Ty: {Ty}')


def main():
    rclpy.init()
    node = turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()

