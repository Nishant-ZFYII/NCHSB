"""
Navigation-only bringup (EKF + SLAM + Nav2).
Assumes sensors are already running via sensors_bringup.launch.py.
Useful for restarting navigation without restarting sensors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_pkg = get_package_share_directory('rc_hardware_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    slam_params = LaunchConfiguration('slam_params')
    nav2_params = LaunchConfiguration('nav2_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(
                hw_pkg, 'config', 'slam_localization_hardware.yaml'),
            description='SLAM params file',
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(
                hw_pkg, 'config', 'nav2_hardware.yaml'),
            description='Nav2 params file',
        ),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_imu_odom',
            output='screen',
            parameters=[
                os.path.join(hw_pkg, 'config', 'ekf_hardware.yaml'),
                {'use_sim_time': False},
            ],
        ),

        # SLAM
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': False}],
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [nav2_bringup_share, 'launch', 'navigation_launch.py']
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
    ])
