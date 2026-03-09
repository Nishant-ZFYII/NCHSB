"""
Build a static map from a rosbag replay using SLAM Toolbox.

Usage:
  ros2 launch rc_hardware_bringup mapping_replay.launch.py \
      bag_path:=/path/to/bag.mcap rate:=1.0

After the bag finishes playing, save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/corridor_map

Then save the SLAM Toolbox pose-graph for localization mode:
  ros2 service call /slam_toolbox/serialize_map \
      slam_toolbox/srv/SerializePoseGraph \
      "{filename: '/home/nishant/maps/corridor_posegraph'}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_pkg = get_package_share_directory('rc_hardware_bringup')

    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')

    replay_urdf_xacro = os.path.join(
        hw_pkg, 'urdf', 'rc_model_replay.urdf.xacro')
    slam_params = os.path.join(hw_pkg, 'config', 'slam_hardware.yaml')

    robot_description = Command(['xacro ', replay_urdf_xacro])

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'MS_Project/NCHSB/rosbags/'
                'rgbd_imu_20260228_003828_0.mcap'),
            description='Path to MCAP rosbag'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Playback speed multiplier'),

        # --- TF tree ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}],
            output='screen',
        ),

        # odom → base_link identity (no wheel encoders in the bag)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'odom', 'base_link'],
            output='screen',
        ),

        # --- Rosbag replay ---
        Node(
            package='rc_hardware_bringup',
            executable='mcap_replayer.py',
            name='mcap_replayer',
            output='screen',
            parameters=[{
                'bag_path': bag_path,
                'rate': rate,
                'loop': False,
            }],
        ),

        # --- SLAM Toolbox (mapping mode) ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': False}],
        ),
    ])
