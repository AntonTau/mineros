cmake_minimum_required(VERSION 3.8)
project(anton_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mineros_inter REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add your executable
add_executable(place_block_node src/anton_place_block_node.cpp)

# Link dependencies to your executable
ament_target_dependencies(place_block_node rclcpp geometry_msgs mineros_inter)

# Install the executable
install(TARGETS place_block_node DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_package()

# add_executable(anton_test_node src/anton_test_node.cpp)
# target_include_directories(anton_test_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(anton_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   anton_test_node
#   "rclcpp"
#   "std_msgs"
#   "mineros_inter"
# )

# install(TARGETS anton_test_node
#   DESTINATION lib/${PROJECT_NAME})

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
