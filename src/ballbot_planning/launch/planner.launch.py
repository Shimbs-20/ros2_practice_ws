from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────────
    #
    # planner: which algorithm to run.
    #   "dijkstra" — original implementation, untouched
    #   "astar"    — new A* implementation (faster, same optimal path)
    #
    # use_python: choose Python or C++ executable.
    #
    # Both planners publish on the SAME topics (/dijkstra/path, /dijkstra/map)
    # so the rest of the stack does not need to change.
    #
    planner_arg = DeclareLaunchArgument(
        "planner",
        default_value="astar",
        description="Path planning algorithm: 'dijkstra' or 'astar'",
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
        description="Use the Python implementation instead of C++",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    planner = LaunchConfiguration("planner")
    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time")

    common_params = [{"use_sim_time": use_sim_time}]

    # ── Dijkstra (C++) ──────────────────────────────────────────────────────
    dijkstra_cpp = Node(
        package="ballbot_planning",
        executable="dijkstra_planner",
        name="dijkstra_planner",
        output="screen",
        parameters=common_params,
        condition=IfCondition(PythonExpression(
            ["'", planner, "' == 'dijkstra' and '", use_python, "' in ('False','false','0')"]
        )),
    )

    # ── Dijkstra (Python) ───────────────────────────────────────────────────
    dijkstra_py = Node(
        package="ballbot_planning",
        executable="dijkstra_planner.py",
        name="dijkstra_planner",
        output="screen",
        parameters=common_params,
        condition=IfCondition(PythonExpression(
            ["'", planner, "' == 'dijkstra' and '", use_python, "' in ('True','true','1')"]
        )),
    )

    # ── A* (C++) ────────────────────────────────────────────────────────────
    astar_cpp = Node(
        package="ballbot_planning",
        executable="a_star_planner",
        name="a_star_planner",
        output="screen",
        parameters=common_params,
        condition=IfCondition(PythonExpression(
            ["'", planner, "' == 'astar' and '", use_python, "' in ('False','false','0')"]
        )),
    )

    # ── A* (Python) ─────────────────────────────────────────────────────────
    astar_py = Node(
        package="ballbot_planning",
        executable="a_star_planner.py",
        name="a_star_planner",
        output="screen",
        parameters=common_params,
        condition=IfCondition(PythonExpression(
            ["'", planner, "' == 'astar' and '", use_python, "' in ('True','true','1')"]
        )),
    )

    return LaunchDescription([
        planner_arg,
        use_python_arg,
        use_sim_time_arg,
        dijkstra_cpp,
        dijkstra_py,
        astar_cpp,
        astar_py,
    ])
