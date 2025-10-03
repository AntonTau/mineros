cmake_minimum_required(VERSION 3.8)
project(anton_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mineros_inter REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "../mineros_interface/srv/BotPos.srv"
  "../mineros_interface/srv/FindBlocks.srv"
  DEPENDENCIES geometry_msgs
)

# Add executables
add_executable(anton_place_block_node src/anton_place_block_node.cpp)

# Link dependencies to each executable
ament_target_dependencies(anton_place_block_node rclcpp geometry_msgs mineros_inter)

# Install the executables
install(TARGETS anton_place_block_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Call environment hooks before ament_package()
ament_environment_hooks()

# Finalize the package
ament_package()

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

