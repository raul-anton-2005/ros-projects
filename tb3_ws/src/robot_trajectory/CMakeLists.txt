cmake_minimum_required(VERSION 3.8)
project(robot_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(square src/square.cpp)
ament_target_dependencies(square rclcpp geometry_msgs)

add_executable(square_odom src/square_odom.cpp)
ament_target_dependencies(square_odom rclcpp geometry_msgs nav_msgs)

add_executable(spiral_inside src/spiral_inside.cpp)
ament_target_dependencies(spiral_inside rclcpp geometry_msgs nav_msgs)

add_executable(polygon_old src/polygon_old.cpp)
ament_target_dependencies(polygon_old rclcpp geometry_msgs nav_msgs)

add_executable(wandering src/wandering.cpp)
ament_target_dependencies(wandering rclcpp geometry_msgs sensor_msgs)

add_executable(polygon src/polygon.cpp)
ament_target_dependencies(polygon rclcpp geometry_msgs nav_msgs)

add_executable(spiral src/spiral.cpp)
ament_target_dependencies(spiral rclcpp geometry_msgs nav_msgs)

install(TARGETS
 square
 square_odom
 spiral_inside
 polygon_old
 wandering
 polygon
 spiral
 DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
