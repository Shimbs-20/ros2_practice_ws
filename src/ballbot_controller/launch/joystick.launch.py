from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

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

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop_node
    ])
