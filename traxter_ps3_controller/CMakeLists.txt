cmake_minimum_required(VERSION 3.8)
project(traxter_ps3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(controller_feedback_node src/controller_feedback.cpp)
ament_target_dependencies(
  controller_feedback_node 
  rclcpp
  sensor_msgs
)

install(TARGETS
  controller_feedback_node
  DESTINATION lib/${PROJECT_NAME}
 )

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
