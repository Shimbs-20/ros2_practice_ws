# Ballbot ROS2 Workspace

ROS2 Jazzy workspace for a differential-drive ballbot with Dijkstra path planning.

## Focus Packages
- `ballbot_controller` — differential drive controller (Python + C++)
- `ballbot_planning` — Dijkstra path planner (Python + C++)
- `ballbot_description` — URDF/Xacro robot model and Gazebo worlds

## Ignore These Packages
- `mc_tut1`, `my_package_tut1`, `cpp_srvcli`, `prpp_tutorial_cpp`

## Build Commands
```bash
cd ~/ros2_ws && colcon build --symlink-install
colcon build --symlink-install --packages-select ballbot_controller
source install/setup.bash
```

## Testing
```bash
colcon test --packages-select <package>
colcon test-result --verbose
```

## Code Conventions
- Python nodes: class-based, inherit from `rclpy.node.Node`
- Use `self.get_logger()` not `print()`
- Declare all params with `self.declare_parameter()` in `__init__`
- Use `#!/usr/bin/env python3` shebang
- No `rospy` — ROS2 only

## Key Facts
- ROS distro: jazzy
- `ballbot_controller` uses TwistStamped (use_stamped_vel: true)
- Wheel separation: 0.33m, Wheel radius: 0.17m
- base_frame_id: base_footprint, odom_frame_id: odom

## Do Not
- Do not modify URDF/Xacro without asking
- Do not add dependencies to package.xml without asking
- Do not use print() — use node logger
- Do not reorder middleware in launch files without asking

## Branch Convention
- Feature branches: `feature/<description>`
- Commit messages: imperative tense ("Add tests for controller")
## Path Planning
- Dijkstra planner: `src/ballbot_planning/ballbot_planning/dijkstra_planner.py`
- Planner subscribes to `/map` (OccupancyGrid) and `/goal_pose` (PoseStamped)
- Publishes path to `/dijkstra/path` and visualization to `/dijkstra/map`
- Current issue: publishes visualization after EVERY cell expansion — very slow
- exploration_directions are 4-connected (no diagonals)
- Grid coordinates via world_grid() and grid_to_world() helpers

## Do Not
- Do not change the topic names without asking
- Do not remove Dijkstra — add A* as an alternative, switchable via ROS parameter
