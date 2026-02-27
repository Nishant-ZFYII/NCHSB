"""
Teleop-only launch for hardware testing.
Brings up sensors + Teensy bridge + keyboard teleop.
No Nav2 / SLAM — just manual driving.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hw_pkg = get_package_share_directory('rc_hardware_bringup')

    serial_port_lidar = LaunchConfiguration('serial_port_lidar')
    serial_port_teensy = LaunchConfiguration('serial_port_teensy')

    xacro_file = os.path.join(hw_pkg, 'urdf', 'rc_model_hardware.urdf.xacro')
    urdf_xml = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port_lidar', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'serial_port_teensy', default_value='/dev/ttyACM0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(urdf_xml, value_type=str),
                'use_sim_time': False,
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hw_pkg, 'launch', 'sensors_bringup.launch.py')
            ),
            launch_arguments={
                'serial_port_lidar': serial_port_lidar,
                'serial_port_teensy': serial_port_teensy,
            }.items(),
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            emulate_tty=True,
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
