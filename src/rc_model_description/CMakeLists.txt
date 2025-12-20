cmake_minimum_required(VERSION 3.8)
project(rc_model_description)

find_package(ament_cmake REQUIRED)

# Install the model files so "package://rc_model_description/..." resolves
install(DIRECTORY urdf meshes launch rviz config worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
