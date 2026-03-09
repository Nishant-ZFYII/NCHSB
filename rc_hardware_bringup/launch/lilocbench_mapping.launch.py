"""
Build a SLAM Toolbox posegraph from the LILocBench mapping sequence.

Usage:
  ros2 launch rc_hardware_bringup lilocbench_mapping.launch.py \
      bag_path:=~/MS_Project/lilocbench/bags/mapping_no_cam_ros2 \
      data_dir:=~/MS_Project/lilocbench \
      rate:=1.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hw_pkg = get_package_share_directory('rc_hardware_bringup')

    bag_path = LaunchConfiguration('bag_path')
    data_dir = LaunchConfiguration('data_dir')
    rate = LaunchConfiguration('rate')

    replay_urdf = os.path.join(hw_pkg, 'urdf', 'rc_model_lilocbench.urdf.xacro')
    slam_params = os.path.join(hw_pkg, 'config', 'slam_lilocbench.yaml')
    robot_desc = ParameterValue(
        Command(['xacro ', replay_urdf]), value_type=str)

    lilocbench_base = os.path.join(
        os.path.expanduser('~'), 'MS_Project', 'lilocbench')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(
                lilocbench_base, 'bags', 'mapping_no_cam_ros2'),
            description='Path to converted MCAP bag directory'),
        DeclareLaunchArgument(
            'data_dir',
            default_value=lilocbench_base,
            description='Path to LILocBench individual files directory'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Playback speed multiplier'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False,
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),
        Node(
            package='rc_hardware_bringup',
            executable='lilocbench_replayer.py',
            name='lilocbench_replayer',
            output='screen',
            parameters=[{
                'bag_path': bag_path,
                'data_dir': data_dir,
                'rate': rate,
                'loop': False,
            }],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': False}],
        ),
    ])
