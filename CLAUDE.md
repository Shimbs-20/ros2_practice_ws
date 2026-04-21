# Ballbot ROS2 Workspace
ROS2 Jazzy workspace for a differential-drive ballbot with Dijkstra path planning.

## Focus Packages
- `ballbot_controller` — differential drive controller (Python + C++)
- `ballbot_planning` — Dijkstra path planner (Python + C++)
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

## Key Facts
- ROS distro: jazzy
- Wheel separation: 0.033m (from URDF), Wheel radius: 0.033m
- base_frame_id: base_footprint, odom_frame_id: odom
- Flask web dashboard runs at http://localhost:8080
- Flask runs as daemon thread inside ROS2 node, not separate process

## Health Monitor
- Package: `ballbot_health_monitor`
- Monitors: /odom, /scan, /imu, /joint_states, /simple_velocity_controller/commands
- Topic timeout threshold: 2 seconds → WARN, 5 seconds → ERROR
- Battery WARN: below 20%, ERROR: below 10%
- Publishes DiagnosticArray to /diagnostics at 1Hz
- Publishes JSON summary to /health_summary (std_msgs/String)
- Tests must run without a live ROS2 system using mocks

## Path Planning
- Dijkstra planner: `src/ballbot_planning/ballbot_planning/dijkstra_planner.py`
- A* planner: `src/ballbot_planning/ballbot_planning/a_star_planner.py`
- Planner subscribes to `/map` and `/goal_pose`
- Publishes path to `/dijkstra/path` and visualization to `/dijkstra/map`

## Do Not
- Do not modify URDF/Xacro without asking
- Do not add dependencies to package.xml without asking first
- Do not use print() — use node logger
- Do not reorder middleware in launch files without asking
- Do not change planner topic names without asking
- Do not remove Dijkstra — A* is the alternative switchable via ROS parameter
- Do not use rosbridge — use Flask only for web dashboard

## Branch Convention
- Feature branches: `feature/<description>`
- Commit messages: imperative tense ("Add health monitor node")
