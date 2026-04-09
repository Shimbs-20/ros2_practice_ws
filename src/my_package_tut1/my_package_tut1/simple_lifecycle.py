from rclpy.lifecycle import Node, State, TransitionCallbackReturn
import rclpy
import rclpy.executors
from std_msgs.msg import String
import time


class Simplelifecycle(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state : State) -> TransitionCallbackReturn:
        self.sub = self.create_subscription(String, "Chatter", self.callback, 10)
        self.get_logger().info("Lifecycle node is configured and Subscribed to chatter")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.destroy_subscription(self.sub)
        self.get_logger().info("Lidecycle node in shutdown and Subscriber is destroyed")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.destroy_subscription(self.sub)
        self.get_logger().info("Lidecycle node in shutdown and Subscriber is destroyed")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Lifecycle node is Activated")
        time.sleep(5)
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info("Lifecycle node is Deactivated")
        return super().on_deactivate(state)


    def callback(self, msg):
        curent_state = self._state_machine.curent_state
        if curent_state[1] == "active":
            self.get_logger().info("I heard: '%s'", msg.data)



def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    simple_lifecycle = Simplelifecycle("simple_lifecycle")
    executor.add_node(Simplelifecycle)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ShutdownException):
        simple_lifecycle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
