cmake_minimum_required(VERSION 3.5)
project(rc_racing_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/racing_line_planner.cpp
)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_common
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

# Link yaml-cpp
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# Export plugin description
pluginlib_export_plugin_description_file(nav2_core racing_line_planner_plugin.xml)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description
install(FILES racing_line_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_common
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

ament_package()

