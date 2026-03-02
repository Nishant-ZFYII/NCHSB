"""
DA3 Costmap Perception Stack launch file.

Launches the perception nodes needed for DA3 depth-based costmap
experiments. Designed to be launched alongside hardware_bringup.launch.py
or with a rosbag replay.

Nodes launched:
  1. da3_to_pointcloud  — converts DA3 depth to PointCloud2 for Nav2
  2. dynamic_inflation  — adjusts Nav2 inflation radius based on YOLO
                          detections (only if strategy != 'fixed')

DA3 inference itself is handled externally:
  - Jetson: GerdsenAI wrapper (~/da3_ros2/run.sh)
  - Local:  python3 -m depth_anything_3.ros2_node (if available)
  - Rosbag: pre-recorded /depth_anything_3/depth topic

Usage:
    # With hardware bringup (DA3 + LiDAR fused costmap):
    ros2 launch rc_hardware_bringup hardware_bringup.launch.py
    ros2 launch rc_hardware_bringup da3_costmap.launch.py

    # With dynamic inflation:
    ros2 launch rc_hardware_bringup da3_costmap.launch.py \\
        inflation_strategy:=class_aware

    # DA3-only (no LiDAR):
    ros2 launch rc_hardware_bringup da3_costmap.launch.py \\
        costmap_config:=nav2_da3_only.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    hw_pkg = get_package_share_directory('rc_hardware_bringup')

    inflation_strategy = LaunchConfiguration('inflation_strategy').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    camera_info = LaunchConfiguration('camera_info_topic').perform(context)
    camera_height = float(LaunchConfiguration('camera_height').perform(context))
    downsample = int(LaunchConfiguration('downsample').perform(context))
    fallback_radius = float(LaunchConfiguration('fallback_radius').perform(context))

    nodes = []

    da3_to_pc = Node(
        package='rc_hardware_bringup',
        executable='da3_to_pointcloud.py',
        name='da3_to_pointcloud',
        output='screen',
        parameters=[{
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info,
            'output_topic': '/da3/pointcloud',
            'output_frame': 'camera_color_optical_frame',
            'min_depth': 0.1,
            'max_depth': 5.0,
            'min_height': 0.05,
            'max_height': 0.50,
            'camera_height': camera_height,
            'downsample': downsample,
        }],
    )
    nodes.append(da3_to_pc)

    if inflation_strategy != 'fixed':
        det_topic = LaunchConfiguration('detection_topic').perform(context)

        dyn_inflation = Node(
            package='rc_hardware_bringup',
            executable='dynamic_inflation.py',
            name='dynamic_inflation',
            output='screen',
            parameters=[{
                'detection_topic': det_topic,
                'depth_topic': depth_topic,
                'strategy': inflation_strategy,
                'fallback_radius': fallback_radius,
                'update_rate': 5.0,
                'min_radius': 0.05,
                'max_radius': 0.30,
                'costmap_node': '/local_costmap/local_costmap',
            }],
        )
        nodes.append(dyn_inflation)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'inflation_strategy', default_value='fixed',
            description='Inflation strategy: fixed, corridor_width, '
                        'min_depth, class_aware'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/depth_anything_3/depth',
            description='DA3 depth image topic'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera info topic for intrinsics'),
        DeclareLaunchArgument(
            'detection_topic',
            default_value='/yolo_node/detections',
            description='YOLO detection topic for class-aware inflation'),
        DeclareLaunchArgument(
            'camera_height', default_value='0.25',
            description='Camera height above ground (metres)'),
        DeclareLaunchArgument(
            'downsample', default_value='4',
            description='Depth image downsample factor for PointCloud2'),
        DeclareLaunchArgument(
            'fallback_radius', default_value='0.09',
            description='Fallback inflation radius (metres)'),
        OpaqueFunction(function=launch_setup),
    ])
