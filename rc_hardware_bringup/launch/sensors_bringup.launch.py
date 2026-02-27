"""
Sensor-only bringup: RPLiDAR A2M8 + Orbbec Femto Bolt.
Run this standalone to verify sensors before full navigation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_lidar = LaunchConfiguration('serial_port_lidar')
    serial_port_teensy = LaunchConfiguration('serial_port_teensy')

    hw_pkg = get_package_share_directory('rc_hardware_bringup')
    teensy_config = os.path.join(hw_pkg, 'config', 'teensy_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port_lidar',
            default_value='/dev/ttyUSB0',
            description='RPLiDAR serial device',
        ),
        DeclareLaunchArgument(
            'serial_port_teensy',
            default_value='/dev/ttyACM0',
            description='Teensy USB serial device',
        ),

        # ---- RPLiDAR A2M8 ----
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port_lidar,
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # ---- Orbbec Femto Bolt (RGB + ToF depth + IMU) ----
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='femto_bolt',
            output='screen',
            parameters=[{
                'camera_name': 'camera',
                'enable_color': True,
                'enable_depth': True,
                'enable_ir': False,
                'enable_imu': True,
                'depth_width': 640,
                'depth_height': 576,
                'color_width': 640,
                'color_height': 480,
                'depth_fps': 30,
                'color_fps': 30,
            }],
        ),

        # ---- Teensy Bridge (VESC + encoders) ----
        Node(
            package='rc_hardware_bringup',
            executable='teensy_bridge.py',
            name='teensy_bridge',
            output='screen',
            parameters=[
                teensy_config,
                {'serial_port': serial_port_teensy},
            ],
        ),
    ])
