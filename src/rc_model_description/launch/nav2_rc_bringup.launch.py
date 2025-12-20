from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

#bringup_launch.py -> the nav2 source bringup launch file

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rc_pkg_share = get_package_share_directory('rc_model_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    nav2_params = PathJoinSubstitution(
        [rc_pkg_share, 'config', 'nav2_params_rc.yaml']
    )

    

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup_share, 'launch', 'navigation_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        nav2_launch,
    ])
