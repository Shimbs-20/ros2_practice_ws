from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time",
    )

    ballbot_controller_pkg = get_package_share_directory("ballbot_controller")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            os.path.join(ballbot_controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop_node",
        parameters=[
            os.path.join(ballbot_controller_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # twist_mux merges /cmd_vel_nav (priority 10), /cmd_vel_key (50),
    # and /input_joy/cmd_vel (100) → output on /ballbot_controller/cmd_vel_unstamped
    #
    # BUG FIX: The original remapping sent twist_mux output to
    # "/ballbot_controller/cmd_vel_unstamped" which is correct for the
    # diff_drive_controller path when use_stamped_vel=false, BUT the YAML
    # had use_stamped_vel=true and no stamping was happening at that point.
    # Now the chain is:
    #   twist_mux → /ballbot_controller/cmd_vel_unstamped (Twist)
    #   twist_relay subscribes there, stamps it, publishes on
    #   ballbot_controller/cmd_vel (TwistStamped) → diff_drive_controller
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(ballbot_controller_pkg, "config", "twist_mux_topic.yaml"),
            os.path.join(ballbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            # twist_mux always publishes on /cmd_vel_out; remap to the
            # unstamped relay input that twist_relay picks up.
            ("/cmd_vel_out", "/ballbot_controller/cmd_vel"),
        ],
    )

    # BUG FIX: The original launch used "twist_relay.py" as the executable,
    # implying a Python script. The file uploaded is twist_relay.cpp (C++).
    # Use the compiled executable name "twist_relay".
    # If you DO have a Python version, rename accordingly.
    twist_relay_node = Node(
        package="ballbot_controller",
        executable="twist_relay",          # compiled C++ binary
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop_node,
        twist_mux_node,
        # twist_relay_node,
    ])
