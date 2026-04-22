cmake_minimum_required(VERSION 3.8)
project(ballbot_health_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})


include_directories(
  include
)

add_executable(health_monitor_node_cpp src/health_monitor_node.cpp)
ament_target_dependencies(health_monitor_node_cpp
  rclcpp diagnostic_msgs nav_msgs sensor_msgs std_msgs)

install(DIRECTORY include
  DESTINATION include
)

install(PROGRAMS
  ${PROJECT_NAME}/health_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  health_monitor_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_health_monitor_node test/test_health_monitor_node.py
    TIMEOUT 60
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
