cmake_minimum_required(VERSION 3.8)
project(ballbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
)

add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp geometry_msgs std_msgs nav_msgs tf2_ros tf2_geometry_msgs)

add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp geometry_msgs std_msgs nav_msgs tf2_ros tf2_geometry_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  ${PROJECT_NAME}/a_star_planner.py
  ${PROJECT_NAME}/dijkstra_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  dijkstra_planner
  a_star_planner
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
