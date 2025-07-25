cmake_minimum_required(VERSION 3.5)
project(follow_wall)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# for clang intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_messages REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(client_follow_wall_node src/client_follow_wall.cpp)
ament_target_dependencies(client_follow_wall_node rclcpp geometry_msgs sensor_msgs custom_messages rclcpp_action)

add_executable(service_position_robot_node src/service_position_robot.cpp)
ament_target_dependencies(service_position_robot_node rclcpp geometry_msgs sensor_msgs custom_messages)

add_executable(action_record_odom_node src/action_record_odom.cpp)
ament_target_dependencies(action_record_odom_node rclcpp custom_messages nav_msgs rclcpp_action geometry_msgs)

INSTALL (TARGETS
  client_follow_wall_node
  service_position_robot_node
  action_record_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

INSTALL (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
