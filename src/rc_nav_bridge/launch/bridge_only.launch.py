from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rc_nav_bridge'),
        'config',
        'stamper.params.yaml',
    )
    return LaunchDescription([
        Node(
            package='rc_nav_bridge',
            executable='stamper',
            name='twist_to_stamped',
            output='screen',
            parameters=[params],
        ),
    ])
