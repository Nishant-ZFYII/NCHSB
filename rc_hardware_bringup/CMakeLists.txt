cmake_minimum_required(VERSION 3.8)
project(rc_hardware_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf params
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    rc_hardware_bringup/teensy_bridge.py
    rc_hardware_bringup/depth_fusion.py
    rc_hardware_bringup/da3_inference_node.py
    rc_hardware_bringup/da3_to_pointcloud.py
    rc_hardware_bringup/dynamic_inflation.py
    rc_hardware_bringup/mcap_replayer.py
    rc_hardware_bringup/costmap_evaluator.py
    rc_hardware_bringup/yolo_node.py
    rc_hardware_bringup/lilocbench_replayer.py
    rc_hardware_bringup/pose_logger.py
    rc_hardware_bringup/db3_replayer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
