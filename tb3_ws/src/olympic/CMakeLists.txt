cmake_minimum_required(VERSION 3.8)
project(olympic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(olympic_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(rings src/rings.cpp)
ament_target_dependencies(rings rclcpp geometry_msgs turtlesim)

add_executable(rings_action_server src/rings_action_server.cpp)
ament_target_dependencies(rings_action_server rclcpp rclcpp_action geometry_msgs turtlesim olympic_interfaces)

add_executable(rings_action_client src/rings_action_client.cpp)
ament_target_dependencies(rings_action_client rclcpp rclcpp_action geometry_msgs turtlesim olympic_interfaces)

install (TARGETS
  rings
  rings_action_server
  rings_action_client
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
