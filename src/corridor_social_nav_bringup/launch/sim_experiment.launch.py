"""
Unified experiment launch file for corridor social navigation trials.

Launches the full stack for a single experiment trial:
  1. Gazebo Ignition Fortress with specified world
  2. Robot spawn + ros2_control controllers
  3. ros_gz_bridge (scan, imu, clock, GT pose, pedestrian poses)
  4. EKF state estimation
  5. Nav2 with specified controller config
  6. Safety shield node (if enabled)
  7. Ground truth people publisher
  8. Ground truth collision detector
  9. Pedestrian driver (moves spawned models along trajectories)
  10. Metrics logger
  11. Goal sender (triggers navigation + monitors completion)

All parameters are exposed as launch arguments so the experiment
runner can configure each trial programmatically.
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_nav2_launch(context, *args, **kwargs):
    """Deferred construction so we can resolve LaunchConfiguration values."""
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    controller_config = LaunchConfiguration('controller_config').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': controller_config,
            'map': map_yaml,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    return [nav2_launch]


def generate_launch_description():
    rc_pkg = FindPackageShare('rc_model_description')
    bringup_pkg = FindPackageShare('corridor_social_nav_bringup')
    social_nav_pkg = FindPackageShare('corridor_social_nav')

    return LaunchDescription([
        # ── World and robot configuration ──
        DeclareLaunchArgument('world', default_value='corridor_straight.sdf'),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('spawn_x', default_value='-9.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.12'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        # ── Navigation goal ──
        DeclareLaunchArgument('goal_x', default_value='9.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),

        # ── Controller and scenario ──
        DeclareLaunchArgument('controller_config',
            default_value=PathJoinSubstitution([
                rc_pkg, 'config', 'nav2_mppi_vanilla.yaml'])),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('enable_shield', default_value='false'),
        DeclareLaunchArgument('scenario_name', default_value='unknown'),
        DeclareLaunchArgument('controller_name', default_value='unknown'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('timeout_s', default_value='120.0'),

        # ── Pedestrian configuration (JSON string) ──
        DeclareLaunchArgument('pedestrians_json', default_value='[]'),

        # ── Output ──
        DeclareLaunchArgument('output_dir', default_value='/tmp/corridor_results'),
        DeclareLaunchArgument('output_file', default_value='trial_metrics.csv'),

        # ── Noise for tracking ablation ──
        DeclareLaunchArgument('tracking_noise_std', default_value='0.0'),

        # ── 1. Gazebo bringup (includes robot spawn) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([rc_pkg, 'launch', 'fortress_bringup.launch.py'])
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'world_name': LaunchConfiguration('world_name'),
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items(),
        ),

        # ── 2. EKF state estimation ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                PathJoinSubstitution([rc_pkg, 'config', 'ekf_imu_odom.yaml']),
                {'use_sim_time': True},
            ],
            output='screen',
        ),

        # ── 3. Nav2 (deferred to resolve config path) ──
        TimerAction(
            period=8.0,
            actions=[OpaqueFunction(function=_build_nav2_launch)],
        ),

        # ── 4. Pedestrian pose bridge (Gazebo -> ROS TF) ──
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ped_pose_bridge',
            arguments=[
                '/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            parameters=[{'use_sim_time': True}],
            output='log',
        ),

        # ── 5. GT people publisher ──
        Node(
            package='corridor_social_nav_bringup',
            executable='gt_people_publisher',
            name='gt_people_publisher',
            parameters=[{
                'use_sim_time': True,
                'noise_std': LaunchConfiguration('tracking_noise_std'),
                'world_name': LaunchConfiguration('world_name'),
            }],
            output='log',
        ),

        # ── 6. GT collision detector ──
        Node(
            package='corridor_social_nav_bringup',
            executable='gt_collision_detector',
            name='gt_collision_detector',
            parameters=[{'use_sim_time': True}],
            output='log',
        ),

        # ── 7. Pedestrian driver ──
        Node(
            package='corridor_social_nav_bringup',
            executable='pedestrian_driver',
            name='pedestrian_driver',
            parameters=[{
                'use_sim_time': True,
                'pedestrians_json': LaunchConfiguration('pedestrians_json'),
                'world_name': LaunchConfiguration('world_name'),
            }],
            output='log',
        ),

        # ── 8. CBF safety shield (conditional) ──
        Node(
            package='corridor_social_nav',
            executable='safety_shield_node',
            name='safety_shield_node',
            parameters=[{'use_sim_time': True}],
            output='log',
            condition=IfCondition(LaunchConfiguration('enable_shield')),
        ),

        # ── 9. Metrics logger ──
        Node(
            package='corridor_social_nav_bringup',
            executable='metrics_logger',
            name='metrics_logger',
            parameters=[{
                'use_sim_time': True,
                'output_file': LaunchConfiguration('output_file'),
                'scenario': LaunchConfiguration('scenario_name'),
                'controller': LaunchConfiguration('controller_name'),
                'seed': LaunchConfiguration('seed'),
                'timeout_s': LaunchConfiguration('timeout_s'),
            }],
            output='screen',
        ),

        # ── 10. Goal sender (delayed to let Nav2 initialize) ──
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='corridor_social_nav_bringup',
                    executable='goal_sender',
                    name='goal_sender',
                    parameters=[{
                        'use_sim_time': True,
                        'goal_x': LaunchConfiguration('goal_x'),
                        'goal_y': LaunchConfiguration('goal_y'),
                        'goal_yaw': LaunchConfiguration('goal_yaw'),
                        'timeout_s': LaunchConfiguration('timeout_s'),
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
