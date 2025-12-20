import os
import xacro # <-- New import
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('rc_model_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rc_model_ros2.xacro')

    # --- THIS IS THE NEW, ROBUST METHOD ---
    # Use the xacro library to process the file
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    # ----------------------------------------

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])