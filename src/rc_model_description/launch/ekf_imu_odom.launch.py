# rc_model_description/launch/ekf_imu_odom.launch.py
# IMU ADDED: launch robot_localization EKF (odom + IMU)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_model_description')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_imu_odom.yaml')

    # Allow overriding use_sim_time from CLI, default true for Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_imu_odom',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
    ])
