import os
import sys
from unittest.mock import MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def _install_ros2_mocks():
    """Install lightweight stand-ins for ROS2 modules so tests run without rclpy."""
    if 'rclpy' in sys.modules and not isinstance(sys.modules['rclpy'], MagicMock):
        return

    rclpy = MagicMock()

    class _Node:
        def __init__(self, name, *args, **kwargs):
            self._name = name

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_publisher(self, *args, **kwargs):
            return MagicMock()

        def create_timer(self, *args, **kwargs):
            return MagicMock()

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            return MagicMock()

        def get_name(self):
            return self._name

    node_mod = MagicMock()
    node_mod.Node = _Node
    rclpy.node = node_mod

    diag_mod = MagicMock()

    class _DiagnosticStatus:
        OK = b'\x00'
        WARN = b'\x01'
        ERROR = b'\x02'
        STALE = b'\x03'

        def __init__(self):
            self.level = self.OK
            self.name = ''
            self.message = ''
            self.hardware_id = ''
            self.values = []

    class _DiagnosticArray:
        def __init__(self):
            self.header = MagicMock()
            self.status = []

    class _KeyValue:
        def __init__(self, key='', value=''):
            self.key = key
            self.value = value

    diag_mod.DiagnosticStatus = _DiagnosticStatus
    diag_mod.DiagnosticArray = _DiagnosticArray
    diag_mod.KeyValue = _KeyValue

    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod
    sys.modules['diagnostic_msgs'] = MagicMock()
    sys.modules['diagnostic_msgs.msg'] = diag_mod
    sys.modules['nav_msgs'] = MagicMock()
    sys.modules['nav_msgs.msg'] = MagicMock()
    sys.modules['sensor_msgs'] = MagicMock()
    sys.modules['sensor_msgs.msg'] = MagicMock()
    sys.modules['std_msgs'] = MagicMock()
    sys.modules['std_msgs.msg'] = MagicMock()


_install_ros2_mocks()
