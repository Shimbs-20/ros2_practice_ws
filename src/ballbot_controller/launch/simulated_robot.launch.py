import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ballbot_description"),
            "launch",
            "gazebo.launch.py",
        ),
    )

    # Delay controller spawning by 5 seconds to let Gazebo
    # fully load and the ros2_control plugin initialize.
    # Without this, the spawner often starts before the
    # controller_manager exists.
    controller = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("ballbot_controller"),
                    "launch",
                    "controller.launch.py",
                ),
                launch_arguments={
                    "use_simple_controller": "False",
                    "use_python": "False",
                }.items(),
            ),
        ],
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ballbot_controller"),
            "launch",
            "joystick.launch.py",
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        controller,
        joystick,
    ])
