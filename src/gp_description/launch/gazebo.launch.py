#!/usr/bin/env python3
# gazebo.launch.py
# Launches Gazebo Harmonic (gz-sim 8.x) with the BallBot URDF.
# Compatible with ROS 2 Jazzy.

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory("gp_description")

    # ── Arguments ────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg, "urdf", "ballbot1.urdf.xacro"),
        description="Absolute path to robot URDF xacro file",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="World name (without .world extension)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
    )

    # ── World path ───────────────────────────────────────────────
    world_path = PathJoinSubstitution([
        pkg, "worlds",
        PythonExpression([
            "'", LaunchConfiguration("world_name"), "' + '.world'"
        ]),
    ])

    # ── Model resource path (for custom meshes / models) ─────────
    model_path = os.pathsep.join([
        str(Path(pkg).parent.resolve()),
        os.path.join(pkg, "models"),
    ])

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path,
    )

    # ── Robot description (xacro → URDF) ─────────────────────────
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    # ── robot_state_publisher ─────────────────────────────────────
    # Publishes static TF transforms from URDF joint tree
    # (base_link → laser_link, base_link → wheel joints etc.)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    # ── Gazebo Harmonic ───────────────────────────────────────────
    # gz_args: world file path + flags
    #   -v 4  = verbose level 4 (errors + warnings + info)
    #   -r    = start simulation running immediately
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, "' + ' -v 4 -r'"]),
        }.items(),
    )

    # ── Spawn robot into Gazebo ───────────────────────────────────
    # Reads robot_description topic published by robot_state_publisher.
    # Delay to ensure robot_description is published before spawning.
    gz_spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_ballbot",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-name",  "BallBot",
                    "-z",     "0.13",   # spawn slightly above ground to avoid collision at t=0
                ],
            ),
        ],
    )

    # ── ROS ↔ Gazebo topic bridge ─────────────────────────────────
    # Bridges Gazebo internal topics to ROS 2 topics.
    #
    # Format: /topic@ros_type[gz_type   (Gazebo → ROS)
    #         /topic@ros_type]gz_type   (ROS → Gazebo)
    #         /topic@ros_type@gz_type   (bidirectional)
    #
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # Simulation clock → ROS (mandatory for use_sim_time)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Lidar scan → ROS (for SLAM)
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # IMU → ROS (for EKF / localization)
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # cmd_vel ROS → Gazebo (needed if using PlanarMove plugin)
            # Uncomment if you add the PlanarMove plugin:
            # "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # Odometry Gazebo → ROS (if using PlanarMove plugin)
            # "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # Joint states Gazebo → ROS (for robot_state_publisher)
            "/world/empty_world/model/BallBot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        use_sim_time_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn,
        gz_bridge,
    ])
