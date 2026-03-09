"""
Costmap ablation: replay a rosbag in SLAM-localization mode and run
Nav2's local costmap with one of four sensor configurations.

Configs (set via `costmap_config` launch arg):
  L    — LiDAR only                (nav2_hardware.yaml)
  L+D  — LiDAR + DA3 depth         (nav2_da3_costmap.yaml)
  L+S  — LiDAR + sensor depth      (nav2_lidar_sensor_depth.yaml)
  D    — DA3 depth only            (nav2_da3_only.yaml)

The launch uses nav2_hardware.yaml as the base params for all Nav2 nodes
(controller, planner, etc.) and overlays the costmap-specific config on
top so that the controller plugin is always correctly configured.

Usage:
  ros2 launch rc_hardware_bringup costmap_ablation.launch.py \
      costmap_config:=L+D \
      bag_path:=/path/to/bag.mcap \
      map_posegraph:=/home/nishant/maps/corridor_posegraph \
      rate:=0.3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, LogInfo, GroupAction)
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _merge_yaml_files(base_path: str, overlay_path: str) -> str:
    """Merge two YAML files: overlay values override base values.

    Returns path to a temporary merged YAML file.
    """
    import yaml
    import tempfile

    def _deep_merge(base: dict, overlay: dict) -> dict:
        result = dict(base)
        for k, v in overlay.items():
            if (k in result and isinstance(result[k], dict)
                    and isinstance(v, dict)):
                result[k] = _deep_merge(result[k], v)
            else:
                result[k] = v
        return result

    with open(base_path) as f:
        base = yaml.safe_load(f)
    with open(overlay_path) as f:
        overlay = yaml.safe_load(f)

    merged = _deep_merge(base, overlay)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_merged_', delete=False)
    yaml.dump(merged, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _build_nodes(context, *args, **kwargs):
    """Resolve the costmap_config arg and return the appropriate node list."""
    hw_pkg = get_package_share_directory('rc_hardware_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    config = context.launch_configurations['costmap_config']
    bag_path = context.launch_configurations['bag_path']
    rate = context.launch_configurations['rate']
    map_pg = context.launch_configurations['map_posegraph']

    config_map = {
        'L':   None,
        'L+D': 'nav2_da3_costmap.yaml',
        'L+S': 'nav2_lidar_sensor_depth.yaml',
        'D':   'nav2_da3_only.yaml',
    }
    if config not in config_map:
        raise RuntimeError(
            f"Unknown costmap_config '{config}'. "
            f"Choose from: {list(config_map.keys())}")

    base_yaml = os.path.join(hw_pkg, 'config', 'nav2_hardware.yaml')
    overlay = config_map[config]
    if overlay is not None:
        nav2_yaml = _merge_yaml_files(
            base_yaml, os.path.join(hw_pkg, 'config', overlay))
    else:
        nav2_yaml = base_yaml

    slam_yaml = os.path.join(hw_pkg, 'config',
                             'slam_localization_hardware.yaml')
    replay_urdf = os.path.join(hw_pkg, 'urdf',
                               'rc_model_replay.urdf.xacro')
    robot_desc = Command(['xacro ', replay_urdf])

    nodes = []

    # --- TF tree ---
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                      'use_sim_time': False}],
        output='screen',
    ))
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    ))

    # --- Rosbag replay (auto-detect format) ---
    replayer_exec = ('db3_replayer.py' if bag_path.endswith('.db3')
                     else 'mcap_replayer.py')
    nodes.append(Node(
        package='rc_hardware_bringup',
        executable=replayer_exec,
        name='bag_replayer',
        output='screen',
        parameters=[{
            'bag_path': bag_path,
            'rate': float(rate),
            'loop': True,
        }],
    ))

    # --- SLAM Toolbox (localization mode) ---
    nodes.append(Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_yaml, {
            'use_sim_time': False,
            'map_file_name': map_pg,
        }],
    ))

    # --- Nav2 (full stack with merged costmap config) ---
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.actions import IncludeLaunchDescription
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch',
                         'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_yaml,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    ))

    # --- Config-specific perception nodes ---
    needs_da3 = config in ('L+D', 'D')
    needs_sensor_depth = config == 'L+S'

    if needs_da3:
        nodes.append(Node(
            package='rc_hardware_bringup',
            executable='da3_inference_node.py',
            name='da3_inference',
            output='screen',
            parameters=[{
                'model_id': 'depth-anything/DA3-SMALL',
                'scale_mode': 'median',
            }],
        ))
        nodes.append(Node(
            package='rc_hardware_bringup',
            executable='da3_to_pointcloud.py',
            name='da3_to_pointcloud',
            output='screen',
            parameters=[{
                'depth_topic': '/depth_anything_3/depth',
                'output_topic': '/da3/pointcloud',
                'min_height': -2.0,
                'max_height': 2.0,
                'camera_height': 0.8,
                'downsample': 2,
            }],
        ))

    if needs_sensor_depth:
        nodes.append(Node(
            package='rc_hardware_bringup',
            executable='da3_to_pointcloud.py',
            name='sensor_depth_to_pointcloud',
            output='screen',
            parameters=[{
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'output_topic': '/sensor_depth/pointcloud',
                'output_frame': 'camera_color_optical_frame',
                'min_height': -2.0,
                'max_height': 2.0,
                'camera_height': 0.8,
                'downsample': 2,
            }],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'costmap_config', default_value='L',
            description='Ablation config: L, L+D, L+S, or D'),
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'MS_Project/NCHSB/rosbags/'
                'rgbd_imu_20260228_003828_0.mcap'),
            description='Path to MCAP rosbag'),
        DeclareLaunchArgument(
            'map_posegraph',
            default_value=os.path.join(
                os.path.expanduser('~'), 'maps', 'corridor_posegraph'),
            description='SLAM Toolbox serialized pose-graph path (no ext)'),
        DeclareLaunchArgument(
            'rate', default_value='0.3',
            description='Playback speed (0.3 recommended for DA3 configs)'),

        OpaqueFunction(function=_build_nodes),
    ])
