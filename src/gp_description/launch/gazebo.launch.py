# import os
# from os import pathsep
# from pathlib import Path
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue


# def generate_launch_description():
#     ballbot_description = get_package_share_directory("ballbot_description")

#     model_arg = DeclareLaunchArgument(
#         name="model",
#         default_value=os.path.join(ballbot_description, "urdf", "ballbot.urdf.xacro"),
#         description="Absolute path to robot urdf file"
#     )

#     world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

#     world_path = PathJoinSubstitution([
#             ballbot_description,
#             "worlds",
#             PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
#         ]
#     )

#     model_path = str(Path(ballbot_description).parent.resolve())
#     model_path += pathsep + os.path.join(get_package_share_directory("ballbot_description"), 'models')

#     gazebo_resource_path = SetEnvironmentVariable(
#         "GZ_SIM_RESOURCE_PATH",
#         model_path
#     )

#     ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
#     is_ignition = "True" if ros_distro == "humble" else "False"

#     robot_description = ParameterValue(Command([
#             "xacro ",
#             LaunchConfiguration("model"),
#             " is_ignition:=",
#             is_ignition
#         ]),
#         value_type=str
#     )

#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{"robot_description": robot_description,
#                      "use_sim_time": True}]
#     )

#     # gazebo = IncludeLaunchDescription(
#     #             PythonLaunchDescriptionSource([os.path.join(
#     #                 get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
#     #             launch_arguments={
#     #                 "gz_args": [world_path, " -v 4 -r"]
#     #             }.items()
#     #          )

#     gazebo = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
#                 launch_arguments={
#                     "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
#                 }.items()
#              )

#     gz_spawn_entity = Node(
#         package="ros_gz_sim",
#         executable="create",
#         output="screen",
#         arguments=["-topic", "robot_description",
#                    "-name", "ballbot"],
#     )

#     gz_ros2_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#             "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
#             "imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
#         ]
#     )

#     # # Build the path to your new YAML file
#     # bridge_config = os.path.join(
#     #     get_package_share_directory("ballbot_description"),
#     #     'config',
#     #     'bridge.yaml'
#     # )

#     # # Tell the bridge to use the YAML file instead of arguments
#     # gz_ros2_bridge = Node(
#     #     package="ros_gz_bridge",
#     #     executable="parameter_bridge",
#     #     parameters=[{
#     #         'config_file': bridge_config,
#     #     }],
#     #     output='screen'
#     # )

# #  Inside your ballbot_description package,
# # create a new folder named config (if you don't have one).
# # Inside that, create a file named bridge.yaml and paste this inside.
# # (Notice I included the /clock topic, which is mandatory for Gazebo simulation time!):
# # YAML

# # - ros_topic_name: "/clock"
# #   gz_topic_name: "/clock"
# #   ros_type_name: "rosgraph_msgs/msg/Clock"
# #   gz_type_name: "gz.msgs.Clock"
# #   direction: GZ_TO_ROS

# # - ros_topic_name: "/scan"
# #   gz_topic_name: "/scan"
# #   ros_type_name: "sensor_msgs/msg/LaserScan"
# #   gz_type_name: "gz.msgs.LaserScan"
# # direction: GZ_TO_ROS

#     return LaunchDescription([
#         model_arg,
#         world_name_arg,
#         gazebo_resource_path,
#         robot_state_publisher_node,
#         gazebo,
#         gz_spawn_entity,
#         gz_ros2_bridge,
#     ])
