import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true")

    nav_pkg = get_package_share_directory("ballbot_navigation")

    costmap_node = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "costmap.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    costmap_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[{
            "node_names":   ["costmap"],
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart":    True,
            "bond_timeout":  0.0,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        costmap_node,
        costmap_lifecycle_manager,
    ])
