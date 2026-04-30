import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true")


    use_sim = LaunchConfiguration("use_sim_time")
    nav_pkg = get_package_share_directory("ballbot_navigation")
    life_cycle_nodes = ["controller_server", "planner_server", "smoother_server"]

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "controller_server.yaml"),
            {"use_sim_time": use_sim},
        ],
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "planner.yaml"),
            {"use_sim_time": use_sim},
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "smoother_server.yaml"),
            {"use_sim_time": use_sim},
        ],
    )



    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": life_cycle_nodes},
            {"use_sim_time": use_sim},
            {"autostart": True},
            {"bond_timeout": 0.0},
            ],
    )

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
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_lifecycle_manager,
    ])
