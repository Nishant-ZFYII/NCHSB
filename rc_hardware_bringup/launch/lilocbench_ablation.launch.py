"""
Costmap ablation for LILocBench dataset.

Same five configurations as the corridor ablation:
  L      — LiDAR only
  L+D    — LiDAR + DA3 depth
  L+S    — LiDAR + sensor depth
  D      — DA3 depth only
  L+D+dyn — LiDAR + DA3 + YOLO dynamic inflation

Uses lilocbench_replayer (reads MCAP bag + individual camera files)
and a LILocBench-specific URDF.  Nav2 config merges the base
nav2_hardware.yaml with the Dingo footprint overlay and the
sensor-specific costmap overlay.

Usage:
  ros2 launch rc_hardware_bringup lilocbench_ablation.launch.py \
      costmap_config:=L+D \
      bag_path:=~/MS_Project/lilocbench/bags/dynamics_0_no_cam_ros2 \
      data_dir:=~/MS_Project/lilocbench \
      map_posegraph:=~/maps/lilocbench_posegraph \
      rate:=0.3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _merge_yaml_files(*paths) -> str:
    """Deep-merge multiple YAML files; later paths override earlier."""
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

    merged = {}
    for p in paths:
        if p is None:
            continue
        with open(p) as f:
            data = yaml.safe_load(f)
        if data:
            merged = _deep_merge(merged, data)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_lilocbench_', delete=False)
    yaml.dump(merged, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _build_nodes(context, *args, **kwargs):
    hw_pkg = get_package_share_directory('rc_hardware_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    config = context.launch_configurations['costmap_config']
    bag_path = context.launch_configurations['bag_path']
    data_dir = context.launch_configurations['data_dir']
    rate = context.launch_configurations['rate']
    map_pg = context.launch_configurations['map_posegraph']

    sensor_overlay_map = {
        'L':       None,
        'L+D':     'nav2_da3_costmap.yaml',
        'L+S':     'nav2_lidar_sensor_depth.yaml',
        'D':       'nav2_da3_only.yaml',
        'L+D+dyn': 'nav2_da3_costmap.yaml',
    }
    if config not in sensor_overlay_map:
        raise RuntimeError(
            f"Unknown costmap_config '{config}'. "
            f"Choose from: {list(sensor_overlay_map.keys())}")

    base_yaml = os.path.join(hw_pkg, 'config', 'nav2_hardware.yaml')
    dingo_yaml = os.path.join(hw_pkg, 'config', 'nav2_lilocbench_base.yaml')
    sensor_overlay = sensor_overlay_map[config]
    sensor_yaml = (os.path.join(hw_pkg, 'config', sensor_overlay)
                   if sensor_overlay else None)

    nav2_yaml = _merge_yaml_files(base_yaml, dingo_yaml, sensor_yaml)

    slam_yaml = os.path.join(
        hw_pkg, 'config', 'slam_lilocbench_localization.yaml')
    replay_urdf = os.path.join(
        hw_pkg, 'urdf', 'rc_model_lilocbench.urdf.xacro')
    robot_desc = ParameterValue(
        Command(['xacro ', replay_urdf]), value_type=str)

    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }],
        output='screen',
    ))

    nodes.append(Node(
        package='rc_hardware_bringup',
        executable='lilocbench_replayer.py',
        name='lilocbench_replayer',
        output='screen',
        parameters=[{
            'bag_path': bag_path,
            'data_dir': data_dir,
            'rate': float(rate),
            'loop': True,
            'camera': 'front',
        }],
    ))

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

    needs_da3 = config in ('L+D', 'D', 'L+D+dyn')
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
                'camera_height': 0.66,
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
                'camera_height': 0.66,
                'downsample': 2,
            }],
        ))

    if config == 'L+D+dyn':
        nodes.append(Node(
            package='rc_hardware_bringup',
            executable='yolo_node.py',
            name='yolo_node',
            output='screen',
        ))
        nodes.append(Node(
            package='rc_hardware_bringup',
            executable='dynamic_inflation.py',
            name='dynamic_inflation',
            output='screen',
        ))

    return nodes


def generate_launch_description():
    lilocbench_base = os.path.join(
        os.path.expanduser('~'), 'MS_Project', 'lilocbench')

    return LaunchDescription([
        DeclareLaunchArgument(
            'costmap_config', default_value='L',
            description='Ablation config: L, L+D, L+S, D, or L+D+dyn'),
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(
                lilocbench_base, 'bags', 'dynamics_0_no_cam_ros2'),
            description='Path to converted MCAP bag directory'),
        DeclareLaunchArgument(
            'data_dir',
            default_value=lilocbench_base,
            description='Path to LILocBench individual files directory'),
        DeclareLaunchArgument(
            'map_posegraph',
            default_value=os.path.join(
                os.path.expanduser('~'), 'maps', 'lilocbench_posegraph'),
            description='SLAM Toolbox posegraph path (no extension)'),
        DeclareLaunchArgument(
            'rate', default_value='0.3',
            description='Playback speed'),

        OpaqueFunction(function=_build_nodes),
    ])
