cmake_minimum_required(VERSION 3.8)
project(stats)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mean src/mean.cpp)
ament_target_dependencies(mean rclcpp std_msgs)

add_executable(median src/median.cpp)
ament_target_dependencies(median rclcpp std_msgs)

add_executable(mode src/mode.cpp)
ament_target_dependencies(mode rclcpp std_msgs)

add_executable(min_max src/min_max.cpp)
ament_target_dependencies(min_max rclcpp std_msgs)

install(TARGETS
  mean
  median
  mode
  min_max
  DESTINATION lib/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
