cmake_minimum_required(VERSION 3.8)
project(fairino_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

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

add_executable(joint_state_bridge src/joint_state_bridge.cpp)
ament_target_dependencies(joint_state_bridge rclcpp sensor_msgs fairino_msgs)

add_executable(follow_joint_trajectory_server src/follow_joint_trajectory_server.cpp)
ament_target_dependencies(follow_joint_trajectory_server rclcpp rclcpp_action control_msgs trajectory_msgs fairino_msgs)

install(TARGETS
  joint_state_bridge
  follow_joint_trajectory_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
