from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"]

    )



    simple_controller = Node(
        package="controller_manager", # here the package name of the controller
        executable="spawner", #here the executable name of the controller
        arguments=["simple_velocity_controller",
                   "--controller-manager",
                   "/controller_manager"]
    )

    return LaunchDescription([
        joint_state_broadcaster
        ,simple_controller

    ])
