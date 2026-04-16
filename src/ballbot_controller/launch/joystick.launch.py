from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    ballbot_controller_pkg = get_package_share_directory("ballbot_controller")

    joy_node =Node(
        package= "joy",
        executable= "joy_node",
        name= "joy_node",
        parameters=[os.path.join(get_package_share_directory("ballbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    joy_teleop_node =Node(
        package= "joy_teleop",
        executable= "joy_teleop",
        name= "joy_teleop_node",
        parameters=[os.path.join(get_package_share_directory("ballbot_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "/ballbot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(ballbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(ballbot_controller_pkg, "config", "twist_mux_topic.yaml"),
            "config_joy": os.path.join(ballbot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),

    )


    twist_relay_node = Node(
        package="ballbot_controller",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop_node,
        twist_mux_launch,
        twist_relay_node
    ])
