cmake_minimum_required(VERSION 3.8)
project(rc_model_description)

find_package(ament_cmake REQUIRED)

# Install the model files so "package://rc_model_description/..." resolves
install(DIRECTORY urdf meshes launch rviz config worlds models maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables (depth_error_injector, etc.)
install(PROGRAMS
  scripts/depth_error_injector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
