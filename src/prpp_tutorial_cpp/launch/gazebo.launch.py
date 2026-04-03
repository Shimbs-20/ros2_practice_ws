import os
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    prpp_description_dir = get_package_share_directory("prpp_tutorial_cpp")

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            prpp_description_dir,
            'urdf',
            'Robot_PRPP.urdf.xacro'
        ),
        description="Absolute path to robot urdf xacro file"
    )

    robot_description = ParameterValue(Command([
        'xacro ',
        LaunchConfiguration('model')
        ]),
        value_type=str
        )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo_source_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(prpp_description_dir).parent.resolve())]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"]
             )
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "prpp"], # FIXED NAME
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    # NEW: Spawn the Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

# NEW: Spawn the Position Controller
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"], # <-- MUST SAY THIS
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_source_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        joint_state_broadcaster_spawner, # Added to Launch
        position_controller_spawner,     # Added to Launch
    ])
