cmake_minimum_required(VERSION 3.8)
project(mc_tut1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
<<<<<<< Updated upstream
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
=======
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
>>>>>>> Stashed changes

include_directories(
  include
)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
#run by this  ros2 run mc_tut1 cpp_node 

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(robot_news_station_client src/robot_news_subscriber.cpp)
ament_target_dependencies(robot_news_station_client rclcpp example_interfaces)

add_executable(robot_news_int_pubsub src/robot_news_int_pubsub.cpp)
ament_target_dependencies(robot_news_int_pubsub rclcpp example_interfaces)

add_executable(robot_news_int src/robot_news_int.cpp)
ament_target_dependencies(robot_news_int rclcpp example_interfaces)


add_executable(server_robot src/server_robot.cpp)
ament_target_dependencies(server_robot rclcpp example_interfaces)

add_executable(client_robot src/client_robot.cpp)
ament_target_dependencies(client_robot rclcpp example_interfaces)

add_executable(hardware_status src/hardware_status.cpp)
ament_target_dependencies(hardware_status rclcpp my_robot_interfaces)

add_executable(simple_turtlesim_kinematics src/simple_turtlesim_kinematics.cpp)
ament_target_dependencies(simple_turtlesim_kinematics rclcpp turtlesim)

add_executable(tf2_kinematics src/tf2_kinematics.cpp)
ament_target_dependencies(tf2_kinematics rclcpp tf2 tf2_ros geometry_msgs my_robot_interfaces)

<<<<<<< Updated upstream
add_executable(simple_lifecycle src/simple_lifecycle.cpp)
ament_target_dependencies(simple_lifecycle rclcpp rclcpp_lifecycle std_msgs)

add_executable(QOS_nodePub src/QOS_nodePub.cpp)
ament_target_dependencies(QOS_nodePub rclcpp rclcpp_lifecycle std_msgs)

add_executable(QOS_nodeSub src/QOS_nodeSub.cpp)
ament_target_dependencies(QOS_nodeSub rclcpp rclcpp_lifecycle std_msgs)



=======

add_executable(simple_lifecycle src/simple_lifecycle.cpp)
ament_target_dependencies(simple_lifecycle rclcpp rclcpp_lifecycle std_msgs)

>>>>>>> Stashed changes
install(TARGETS
  cpp_node
  robot_news_station
  robot_news_station_client
  robot_news_int_pubsub
  robot_news_int
  server_robot
  client_robot
  hardware_status
  tf2_kinematics
  simple_turtlesim_kinematics
<<<<<<< Updated upstream
  QOS_nodePub
  simple_lifecycle
  QOS_nodeSub
=======
  simple_lifecycle
>>>>>>> Stashed changes
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
