#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, JointState
from std_msgs.msg import Float64MultiArray

WARN_TIMEOUT_S = 2.0
ERROR_TIMEOUT_S = 5.0

MONITORED_TOPICS = {
    "/odom": Odometry,
    "/scan": LaserScan,
    "/imu": Imu,
    "/joint_states": JointState,
    "/simple_velocity_controller/commands": Float64MultiArray,
}


def classify_timeout(last_seen, now,
                     warn_threshold=WARN_TIMEOUT_S,
                     error_threshold=ERROR_TIMEOUT_S):
    """Return (DiagnosticStatus.level, message) for a topic given its last-seen time.

    last_seen: float seconds, or None if never received.
    now:       float seconds.
    """
    if last_seen is None:
        return DiagnosticStatus.ERROR, "No message received"
    age = now - last_seen
    if age > error_threshold:
        return DiagnosticStatus.ERROR, f"No message for {age:.2f} s"
    if age > warn_threshold:
        return DiagnosticStatus.WARN, f"No message for {age:.2f} s"
    return DiagnosticStatus.OK, f"Last message {age:.2f} s ago"


class HealthMonitorNode(Node):

    def __init__(self):
        super().__init__("health_monitor_node")

        self.last_seen_ = {topic: None for topic in MONITORED_TOPICS}

        self.subs_ = []
        for topic, msg_type in MONITORED_TOPICS.items():
            self.subs_.append(
                self.create_subscription(
                    msg_type, topic, self._make_callback(topic), 10))

        self.diag_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)

        self.timer_ = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info(
            f"Monitoring topics: {list(MONITORED_TOPICS.keys())}")

    def _make_callback(self, topic):
        def _cb(_msg):
            self.last_seen_[topic] = self._now_seconds()
        return _cb

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_diagnostics(self):
        now = self._now_seconds()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        for topic, last_seen in self.last_seen_.items():
            level, message = classify_timeout(last_seen, now)
            status = DiagnosticStatus()
            status.level = level
            status.name = f"{self.get_name()}: {topic}"
            status.message = message
            status.hardware_id = topic
            age_str = "never" if last_seen is None else f"{now - last_seen:.3f}"
            status.values = [KeyValue(key="age_s", value=age_str)]
            arr.status.append(status)

        self.diag_pub_.publish(arr)


def main():
    rclpy.init()
    node = HealthMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
