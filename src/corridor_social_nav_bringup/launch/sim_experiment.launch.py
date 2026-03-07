"""
Unified experiment launch file for corridor social navigation trials.

Launches the full stack for a single experiment trial:
  1. Gazebo Ignition Fortress with specified world
     (includes robot spawn, ros2_control, ros_gz_bridge, EKF)
  2. Spawn pedestrian cylinder models into Gazebo from JSON config
  3. Nav2 with specified controller config
  4. Pedestrian pose bridge (Gazebo -> ROS)
  5. Ground truth people publisher
  6. Ground truth collision detector
  7. Pedestrian driver (moves spawned models along trajectories)
  8. CBF safety shield node (if enabled)
  9. Metrics logger
  10. Goal sender (triggers navigation + monitors completion)

Note: EKF is launched inside fortress_bringup.launch.py, NOT here.

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
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _spawn_pedestrians(context, *args, **kwargs):
    """Spawn pedestrian cylinder models into Gazebo from JSON config."""
    ped_json = LaunchConfiguration('pedestrians_json').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    rc_share = get_package_share_directory('rc_model_description')
    model_sdf = os.path.join(rc_share, 'models', 'pedestrian', 'model.sdf')

    peds = json.loads(ped_json)
    actions = []
    for ped in peds:
        pid = ped['id']
        wp = ped['waypoints']
        sx, sy = wp[0][0], wp[0][1]
        sz = ped.get('spawn_z', 0.85)
        actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', world_name,
                '-file', model_sdf,
                '-name', f'pedestrian_{pid}',
                '-x', str(sx), '-y', str(sy), '-z', str(sz),
            ],
            output='log',
        ))
    return actions


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

        # ── 2. Spawn pedestrian models into Gazebo (after world loads) ──
        TimerAction(
            period=5.0,
            actions=[OpaqueFunction(function=_spawn_pedestrians)],
        ),

        # ── 3. Nav2 (deferred to let Gazebo + controllers start) ──
        TimerAction(
            period=15.0,
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
                'pedestrians_json': ParameterValue(
                    LaunchConfiguration('pedestrians_json'), value_type=str),
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
        # on_exit=Shutdown: when metrics_logger exits (after writing CSV),
        # launch shuts down all processes. No manual Ctrl+C needed.
        # The "Cannot shutdown a ROS adapter" error on exit is cosmetic.
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
            on_exit=Shutdown(reason='Trial complete -- metrics written'),
        ),

        # ── 10. Goal sender (delayed to let Nav2 initialize) ──
        TimerAction(
            period=45.0,
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
