#!/usr/bin/env python3

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, TimerAction
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():

#     pkg_name = 'prpp_tutorial_cpp'
#     pkg_share = get_package_share_directory(pkg_name)
#     urdf_file = os.path.join(pkg_share, 'urdf', 'Robot_PRPP.urdf.xacro')

#     # Declare arguments
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     # Robot State Publisher Node
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
#             'publish_frequency': 30.0
#         }]
#     )

#     # Joint State Publisher GUI Node
#     joint_state_publisher_gui = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         name='joint_state_publisher_gui',
#         output='screen'
#     )

#     # RViz Node - delayed to ensure robot_state_publisher is ready
#     rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
#         parameters=[{
#             'use_sim_time': use_sim_time
#         }]
#     )

#     # Delay RViz startup
#     delayed_rviz = TimerAction(
#         period=2.0,
#         actions=[rviz]
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'),
#         robot_state_publisher,
#         joint_state_publisher_gui,
#         delayed_rviz
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            get_package_share_directory('prpp_tutorial_cpp'),'urdf','Robot_PRPP.urdf.xacro'),
        description="Absolute path to robot urdf xacro file"
        )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('prpp_tutorial_cpp'), 'rviz', 'display.rviz')]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])

