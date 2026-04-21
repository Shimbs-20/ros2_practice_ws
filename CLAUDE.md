# Ballbot ROS2 Workspace
ROS2 Jazzy workspace for a differential-drive ballbot with path planning and health monitoring.

## Focus Packages
- `ballbot_controller` — differential drive controller (Python + C++)
- `ballbot_planning` — Dijkstra and A* path planner (Python + C++)
- `ballbot_description` — URDF/Xacro robot model and Gazebo worlds
- `ballbot_health_monitor` — ROS2 health monitoring node with web dashboard

## Ignore These Packages
- `mc_tut1`, `my_package_tut1`, `cpp_srvcli`, `prpp_tutorial_cpp`

## Build Commands
```bash
cd ~/ros2_ws && colcon build --symlink-install
colcon build --symlink-install --packages-select ballbot_health_monitor
source install/setup.bash
```

## Testing
```bash
colcon test --packages-select ballbot_health_monitor
colcon test-result --verbose
python3 -m pytest src/ballbot_health_monitor/test/ -v
```

## Code Conventions
- Python nodes: class-based, inherit from `rclpy.node.Node`
- Use `self.get_logger()` not `print()`
- Declare all params with `self.declare_parameter()` in `__init__`
- Use `#!/usr/bin/env python3` shebang
- No `rospy` — ROS2 only
- Extract pure logic functions outside the class for testability

## Key Facts
- ROS distro: jazzy
- Wheel radius: 0.033m, Wheel separation: 0.14m (from URDF)
- base_frame_id: base_footprint, odom_frame_id: odom
- Flask web dashboard runs at http://localhost:8080
- Flask runs as daemon thread inside ROS2 node, not separate process

## Health Monitor — Core
- Package: `ballbot_health_monitor`
- Node file: `src/ballbot_health_monitor/ballbot_health_monitor/health_monitor_node.py`
- Monitored topics: /odom, /scan, /imu, /joint_states, /simple_velocity_controller/commands
- Topic timeout: >2s → WARN, >5s → ERROR, never received → ERROR
- Publishes DiagnosticArray to /diagnostics at 1Hz
- Publishes JSON summary to /health_summary (std_msgs/String)
- Tests in: `src/ballbot_health_monitor/test/`
- conftest.py must exist in test directory for pytest discoverability
- Tests must run without live ROS2 using mocks

## Health Monitor — STM32 Checks (Turn 2)
- Battery topic: /battery_state (std_msgs/Float32), value is percentage 0-100
- Battery WARN: below 20%, ERROR: below 10%
- Motor stall detection: ERROR if /simple_velocity_controller/commands is non-zero
  but /joint_states wheel velocities are zero for more than 3 seconds
- Encoder check: both wheel_left_joint and wheel_right_joint must report
  non-zero velocity when commanded — use joint_states.velocity field
- All STM32 checks must be separate DiagnosticStatus entries in the array
- New checks must have their own pytest tests using the same mock pattern

## Do Not
- Do not modify URDF/Xacro without asking
- Do not add dependencies to package.xml without asking first
- Do not use print() — use node logger
- Do not reorder middleware in launch files without asking
- Do not change topic names without asking
- Do not remove Dijkstra — A* is alternative switchable via ROS parameter
- Do not use rosbridge — use Flask only for web dashboard
- Do not use threading.Thread without daemon=True

## Branch Convention
- Feature branches: `feature/<description>`
- Commit messages: imperative tense ("Add STM32 health checks to monitor")
