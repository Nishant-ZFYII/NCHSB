"""
Depth error injection experiment launch file.

Launches the full stack for a single depth-error trial:
  1. Gazebo Ignition Fortress with corridor world + RGBD camera
  2. Depth error injector (applies noise profile to GT depth)
  3. da3_to_pointcloud (converts injected depth to PointCloud2)
  4. Nav2 (MPPI + ObstacleLayer consuming both LiDAR + depth PointCloud2)
  5. Metrics logger (records trial outcome)
  6. Goal sender (sends nav goal, monitors completion)

Pipeline:
  Gazebo GT depth → depth_error_injector → da3_to_pointcloud → Nav2 ObstacleLayer
                         ↑ profile param (0-9)

Usage:
  ros2 launch rc_model_description sim_depth_experiment.launch.py \
      world:=corridor_narrow.sdf depth_profile:=1 \
      goal_x:=6.0 goal_y:=0.0
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_nav2_launch(context, *args, **kwargs):
    """Deferred Nav2 construction: patches AMCL initial_pose with spawn coords
    and auto-resolves the map YAML from the world file name."""
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    rc_share = get_package_share_directory('rc_model_description')

    controller_config = LaunchConfiguration('controller_config').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)

    if not map_yaml:
        world_file = LaunchConfiguration('world').perform(context)
        world_stem = os.path.splitext(os.path.basename(world_file))[0]
        candidate = os.path.join(rc_share, 'maps', f'{world_stem}.yaml')
        if os.path.exists(candidate):
            map_yaml = candidate
            print(f'[sim_depth_experiment] Auto-resolved map: {map_yaml}')
        else:
            print(f'[sim_depth_experiment] WARNING: no map at {candidate}')

    with open(controller_config, 'r') as f:
        content = f.read()

    content = re.sub(
        r'(initial_pose:\s*\n\s+x:\s*)[\d.\-]+',
        r'\g<1>' + spawn_x, content)
    content = re.sub(
        r'(initial_pose:\s*\n\s+x:\s*[\d.\-]+\s*\n\s+y:\s*)[\d.\-]+',
        r'\g<1>' + spawn_y, content)
    content = re.sub(
        r'(initial_pose:\s*\n\s+x:\s*[\d.\-]+\s*\n\s+y:\s*[\d.\-]+\s*\n\s+z:\s*[\d.\-]+\s*\n\s+yaw:\s*)[\d.\-]+',
        r'\g<1>' + spawn_yaw, content)

    patched = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_depth_sim_', delete=False)
    patched.write(content)
    patched.close()

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': patched.name,
            'map': map_yaml,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    return [nav2_launch]


def generate_launch_description():
    rc_pkg = FindPackageShare('rc_model_description')

    return LaunchDescription([
        # ── World and robot ──
        DeclareLaunchArgument('world', default_value='corridor_narrow.sdf'),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('spawn_x', default_value='-6.5'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.12'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),

        # ── Depth error injection ──
        DeclareLaunchArgument('depth_profile', default_value='0',
            description='Error profile: 0=GT, 1=DA3-Small, 2=V4, 3=V5, '
                        '4=V6, 5=V7, 6=V8, 7=V9, 8=sensor-fail, 9=DA3+sensor-fail'),

        # ── Navigation goal ──
        DeclareLaunchArgument('goal_x', default_value='6.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('timeout_s', default_value='120.0'),

        # ── Nav2 config ──
        DeclareLaunchArgument('controller_config',
            default_value=PathJoinSubstitution([
                rc_pkg, 'config', 'nav2_mppi_depth_sim.yaml'])),
        DeclareLaunchArgument('map_yaml', default_value=''),

        # ── Output ──
        DeclareLaunchArgument('output_dir', default_value='/tmp/depth_sim_results'),
        DeclareLaunchArgument('output_file', default_value='depth_trial_metrics.csv'),
        DeclareLaunchArgument('scenario_name', default_value='depth_error_sim'),
        DeclareLaunchArgument('controller_name', default_value='mppi_depth_sim'),
        DeclareLaunchArgument('seed', default_value='0'),

        # ── 1. Gazebo bringup (robot + cameras + error injector) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    rc_pkg, 'launch', 'fortress_bringup.launch.py'])
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'world_name': LaunchConfiguration('world_name'),
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                'gui': LaunchConfiguration('gui'),
                'enable_cameras': 'true',
                'depth_profile': LaunchConfiguration('depth_profile'),
            }.items(),
        ),

        # ── 2. Depth-to-PointCloud2 converter ──
        # Subscribes to injected depth (from depth_error_injector),
        # publishes PointCloud2 for Nav2 ObstacleLayer.
        Node(
            package='rc_hardware_bringup',
            executable='da3_to_pointcloud.py',
            name='da3_to_pointcloud',
            output='screen',
            parameters=[{
                'depth_topic': '/camera/depth_injected',
                'camera_info_topic': '/camera/color/camera_info',
                'output_topic': '/depth/pointcloud',
                'output_frame': 'camera_depth_optical_frame',
                'min_depth': 0.3,
                'max_depth': 5.0,
                'min_height': 0.05,
                'max_height': 0.50,
                'camera_height': 0.20,
                'downsample': 2,
                'use_sim_time': True,
            }],
        ),

        # ── 3. Nav2 (delayed to let Gazebo + controllers start) ──
        TimerAction(
            period=35.0,
            actions=[OpaqueFunction(function=_build_nav2_launch)],
        ),

        # ── 4. Metrics logger ──
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

        # ── 5. Goal sender (delayed for Nav2 to initialize) ──
        TimerAction(
            period=55.0,
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
