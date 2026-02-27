"""
Full hardware bringup for the Traxxas Maxx 4S Ackermann RC car.

Replaces NCHSB's fortress_bringup.launch.py (Gazebo simulation) with
real sensor drivers and the Teensy serial bridge.

Launch order:
  1. robot_state_publisher  (hardware URDF, no Gazebo plugins)
  2. RPLiDAR A2M8           (/scan)
  3. Orbbec Femto Bolt      (/camera/*, /camera/imu)
  4. Teensy Bridge           (/odom, /joint_states, /steering_angle)
  5. EKF                    (/odometry/filtered, odom→base_link TF)
  6. SLAM Toolbox           (map→odom TF)
  7. Nav2                   (planning + control)
  8. Stamper bridge         (/cmd_vel → Teensy understands Twist)
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_pkg = get_package_share_directory('rc_hardware_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # ---- Launch arguments ----
    serial_port_lidar = LaunchConfiguration('serial_port_lidar')
    serial_port_teensy = LaunchConfiguration('serial_port_teensy')
    map_yaml = LaunchConfiguration('map_yaml')
    slam_mode = LaunchConfiguration('slam_mode')

    declare_lidar_port = DeclareLaunchArgument(
        'serial_port_lidar', default_value='/dev/ttyUSB0')
    declare_teensy_port = DeclareLaunchArgument(
        'serial_port_teensy', default_value='/dev/ttyACM0')
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value='',
        description='Map YAML for Nav2 map_server (leave empty to skip)')
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode', default_value='localization',
        description='SLAM mode: mapping | localization')

    # ---- 1. Robot State Publisher (hardware URDF) ----
    xacro_file = os.path.join(hw_pkg, 'urdf', 'rc_model_hardware.urdf.xacro')
    urdf_xml = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(urdf_xml, value_type=str),
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ---- 2. Sensors (RPLiDAR + Femto Bolt + Teensy) ----
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_pkg, 'launch', 'sensors_bringup.launch.py')
        ),
        launch_arguments={
            'serial_port_lidar': serial_port_lidar,
            'serial_port_teensy': serial_port_teensy,
        }.items(),
    )

    # ---- 3. EKF (wheel odom + Femto Bolt IMU) ----
    ekf_config = os.path.join(hw_pkg, 'config', 'ekf_hardware.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_imu_odom',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}],
    )

    # ---- 4. SLAM Toolbox ----
    slam_config_mapping = os.path.join(
        hw_pkg, 'config', 'slam_hardware.yaml')
    slam_config_loc = os.path.join(
        hw_pkg, 'config', 'slam_localization_hardware.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_loc, {'use_sim_time': False}],
    )

    # ---- 5. Nav2 ----
    nav2_params = os.path.join(hw_pkg, 'config', 'nav2_hardware.yaml')

    nav2_launch = IncludeLaunchDescription(
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
    )

    # ---- 6. Stamper bridge (cmd_vel Twist → Teensy understands) ----
    stamper = Node(
        package='rc_nav_bridge',
        executable='stamper',
        name='twist_to_stamped',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'cmd_vel_topic': '/cmd_vel',
            'reference_topic': '/cmd_vel',
        }],
    )

    # ---- 7. Depth fusion (optional, for ToF failure recovery) ----
    depth_fusion = Node(
        package='rc_hardware_bringup',
        executable='depth_fusion.py',
        name='depth_fusion',
        output='screen',
        parameters=[{
            'confidence_threshold': 0.5,
            'trigger_fraction': 0.05,
        }],
    )

    # ---- Staged launch: sensors first, then perception, then nav ----
    delayed_ekf = TimerAction(period=2.0, actions=[ekf_node])
    delayed_slam = TimerAction(period=3.0, actions=[slam_node])
    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])
    delayed_stamper = TimerAction(period=7.0, actions=[stamper])
    delayed_depth = TimerAction(period=3.0, actions=[depth_fusion])

    return LaunchDescription([
        declare_lidar_port,
        declare_teensy_port,
        declare_map_yaml,
        declare_slam_mode,
        robot_state_publisher,
        sensors_launch,
        delayed_ekf,
        delayed_slam,
        delayed_nav2,
        delayed_stamper,
        delayed_depth,
    ])
