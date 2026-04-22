#!/usr/bin/env python3
"""Unit tests for HealthMonitorNode timeout classification.

These tests run without a live ROS2 system: rclpy and message packages are
mocked in conftest.py so only the pure logic and node wiring are exercised.
"""
from unittest.mock import MagicMock, patch

import pytest

from diagnostic_msgs.msg import DiagnosticStatus

from ballbot_health_monitor.health_monitor_node import (
    classify_timeout,
    HealthMonitorNode,
    MONITORED_TOPICS,
    WARN_TIMEOUT_S,
    ERROR_TIMEOUT_S,
)


# ---------------------------------------------------------------------------
# classify_timeout — pure function boundary tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("last_seen,now,expected_level", [
    (None,  0.0,   DiagnosticStatus.ERROR),
    (None,  100.0, DiagnosticStatus.ERROR),
    (10.0,  10.0,  DiagnosticStatus.OK),     # age 0.0
    (10.0,  11.0,  DiagnosticStatus.OK),     # age 1.0
    (10.0,  12.0,  DiagnosticStatus.OK),     # age 2.0 == warn threshold (not >)
    (10.0,  12.01, DiagnosticStatus.WARN),   # just over warn threshold
    (10.0,  13.0,  DiagnosticStatus.WARN),   # age 3.0
    (10.0,  15.0,  DiagnosticStatus.WARN),   # age 5.0 == error threshold (not >)
    (10.0,  15.01, DiagnosticStatus.ERROR),  # just over error threshold
    (10.0,  20.0,  DiagnosticStatus.ERROR),  # age 10.0
])
def test_classify_timeout_boundaries(last_seen, now, expected_level):
    level, message = classify_timeout(last_seen, now)
    assert level == expected_level
    assert isinstance(message, str) and message


def test_classify_timeout_never_received_message():
    level, message = classify_timeout(None, 50.0)
    assert level == DiagnosticStatus.ERROR
    assert "No message received" in message


def test_classify_timeout_custom_thresholds():
    assert classify_timeout(0.0, 0.5, warn_threshold=1.0, error_threshold=3.0)[0] \
        == DiagnosticStatus.OK
    assert classify_timeout(0.0, 1.5, warn_threshold=1.0, error_threshold=3.0)[0] \
        == DiagnosticStatus.WARN
    assert classify_timeout(0.0, 3.5, warn_threshold=1.0, error_threshold=3.0)[0] \
        == DiagnosticStatus.ERROR


def test_default_thresholds():
    assert WARN_TIMEOUT_S == pytest.approx(2.0)
    assert ERROR_TIMEOUT_S == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# HealthMonitorNode — node wiring with mocked rclpy
# ---------------------------------------------------------------------------

@pytest.fixture
def node():
    n = HealthMonitorNode()
    n.diag_pub_ = MagicMock()
    return n


def test_node_tracks_all_monitored_topics(node):
    assert set(node.last_seen_.keys()) == set(MONITORED_TOPICS.keys())
    assert all(v is None for v in node.last_seen_.values())


def test_monitored_topics_are_the_required_set():
    assert set(MONITORED_TOPICS.keys()) == {
        "/odom",
        "/scan",
        "/imu",
        "/joint_states",
        "/simple_velocity_controller/commands",
    }


def test_callback_updates_last_seen(node):
    with patch.object(node, "_now_seconds", return_value=42.0):
        cb = node._make_callback("/odom")
        cb(MagicMock())
    assert node.last_seen_["/odom"] == 42.0
    assert node.last_seen_["/scan"] is None


def test_publish_diagnostics_never_received_all_error(node):
    with patch.object(node, "_now_seconds", return_value=100.0):
        node.publish_diagnostics()

    node.diag_pub_.publish.assert_called_once()
    arr = node.diag_pub_.publish.call_args[0][0]
    assert len(arr.status) == len(MONITORED_TOPICS)
    for st in arr.status:
        assert st.level == DiagnosticStatus.ERROR
        assert st.hardware_id in MONITORED_TOPICS


def test_publish_diagnostics_mixed_levels(node):
    node.last_seen_["/odom"] = 99.5          # 0.5s old  -> OK
    node.last_seen_["/scan"] = 97.0          # 3.0s old  -> WARN
    node.last_seen_["/imu"] = 90.0           # 10.0s old -> ERROR
    # /joint_states, /simple_velocity_controller/commands -> never -> ERROR

    with patch.object(node, "_now_seconds", return_value=100.0):
        node.publish_diagnostics()

    arr = node.diag_pub_.publish.call_args[0][0]
    levels = {st.hardware_id: st.level for st in arr.status}

    assert levels["/odom"] == DiagnosticStatus.OK
    assert levels["/scan"] == DiagnosticStatus.WARN
    assert levels["/imu"] == DiagnosticStatus.ERROR
    assert levels["/joint_states"] == DiagnosticStatus.ERROR
    assert levels["/simple_velocity_controller/commands"] == DiagnosticStatus.ERROR


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
