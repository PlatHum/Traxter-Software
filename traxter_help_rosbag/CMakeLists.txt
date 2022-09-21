cmake_minimum_required(VERSION 3.8)
project(traxter_help_rosbag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(traxter_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(helpwithscan src/helpwithscan.cpp)
add_executable(helpwithwheelspeed src/helpwithwheelspeed.cpp)

ament_target_dependencies(helpwithscan rclcpp sensor_msgs tf2 tf2_ros)
ament_target_dependencies(helpwithwheelspeed rclcpp geometry_msgs tf2 tf2_ros traxter_msgs)

install(TARGETS
   helpwithscan
   helpwithwheelspeed
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
