from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rc_model_description')

    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(
            pkg_share, 'config', 'mapper_params_localization.yaml'
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Full path to slam_toolbox localization params'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        slam_node,
    ])
