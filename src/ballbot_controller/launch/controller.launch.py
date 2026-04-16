from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument , GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )


    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="false",
        description="Use Python controller",
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.33",
        description="Wheel radius",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
        description="Wheel separation distance",
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )


    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )

    # FIXED: Renamed these variables so they don't overwrite the arguments
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")


    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"]
    )

    wheel_simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ballbot_controller",
                   "--controller-manager",
                   "/controller_manager"],
        condition = UnlessCondition(use_simple_controller)

    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller",
                        "--controller-manager",
                        "/controller_manager"]
                ),

            Node(
                package="ballbot_controller",
                executable="simple_controller.py",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation}],
                condition=IfCondition(use_python)
                ),

            Node(
                package="ballbot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation}],
                condition=UnlessCondition(use_python)
                )
        ]
    )



    return LaunchDescription([
        use_python_arg,
        use_simple_controller_arg,
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster,
        simple_controller,
        wheel_simple_controller
    ])
