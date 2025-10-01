cmake_minimum_required(VERSION 3.8)
project(my_first_behaviourtree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mineros_inter REQUIRED)

add_library(
  test_libs
  src/current_position.cpp
  src/determine_next_position.cpp
  src/move_to.cpp
)

target_include_directories(test_libs PUBLIC include)

add_executable(
  launch
  launch.cpp
)

target_link_libraries(launch
  test_libs
)

ament_target_dependencies(test_libs
  rclcpp
  behaviortree_ros2
  behaviortree_cpp
  geometry_msgs
  std_msgs
  mineros_inter
)

ament_target_dependencies(launch
  rclcpp
  behaviortree_ros2
  behaviortree_cpp
  geometry_msgs
  std_msgs
  mineros_inter
)



install(TARGETS launch
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
