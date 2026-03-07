cmake_minimum_required(VERSION 3.8)
project(corridor_social_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pluginlib REQUIRED)
find_package(osqp_vendor REQUIRED)

# ---------- Message Generation ----------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedPerson.msg"
  "msg/TrackedPeople.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ---------- Corridor Frame Library ----------
add_library(corridor_frame SHARED
  src/corridor_frame.cpp
)
target_include_directories(corridor_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corridor_frame
  rclcpp
  nav_msgs
  geometry_msgs
)

# ---------- Social Costmap Layer (instantaneous baseline) ----------
add_library(social_layer SHARED
  src/social_layer.cpp
)
target_include_directories(social_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(social_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(social_layer_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(social_layer ${social_layer_typesupport})

# ---------- Predictive Social Costmap Layer (corridor-frame) ----------
add_library(predictive_social_layer SHARED
  src/predictive_social_layer.cpp
)
target_include_directories(predictive_social_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(predictive_social_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
)
rosidl_get_typesupport_target(pred_layer_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(predictive_social_layer ${pred_layer_typesupport} corridor_frame)

# ---------- Safety Shield CBF Library (no ROS dependency, testable) ----------
add_library(safety_shield_cbf SHARED
  src/safety_shield_cbf.cpp
)
target_include_directories(safety_shield_cbf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(safety_shield_cbf osqp::osqp)

# ---------- Safety Shield Node ----------
add_executable(safety_shield_node
  src/safety_shield_node.cpp
)
target_include_directories(safety_shield_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_shield_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)
rosidl_get_typesupport_target(shield_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(safety_shield_node ${shield_typesupport} safety_shield_cbf)

# ---------- Install ----------
install(TARGETS
  corridor_frame
  social_layer
  predictive_social_layer
  safety_shield_cbf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS safety_shield_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_description.xml)

# ---------- Tests ----------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_safety_shield_cbf test/test_safety_shield_cbf.cpp)
  target_include_directories(test_safety_shield_cbf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_safety_shield_cbf safety_shield_cbf)

  ament_add_gtest(test_corridor_frame test/test_corridor_frame.cpp)
  target_include_directories(test_corridor_frame PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_corridor_frame nav_msgs geometry_msgs)
  target_link_libraries(test_corridor_frame corridor_frame)

  ament_add_gtest(test_social_costmap_math test/test_social_costmap_math.cpp)
  target_include_directories(test_social_costmap_math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_social_costmap_math nav_msgs geometry_msgs)
  target_link_libraries(test_social_costmap_math corridor_frame)
endif()

ament_package()
