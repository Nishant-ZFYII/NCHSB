from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Topics
    cmd_vel     = LaunchConfiguration('cmd_vel_topic')
    reference   = LaunchConfiguration('reference_topic')
    frame_id    = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic',     default_value='/cmd_vel'),
        DeclareLaunchArgument('reference_topic',   default_value='/ackermann_steering_controller/reference'),
        DeclareLaunchArgument('frame_id',          default_value=''),

        # Keyboard teleop (publishes Twist to /cmd_vel)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/cmd_vel', cmd_vel),
            ],
        ),

        # Stamper: /cmd_vel (Twist) -> /ackermann_steering_controller/reference (TwistStamped)
        Node(
            package='rc_nav_bridge',
            executable='stamper',
            name='twist_to_stamped',
            output='screen',
            parameters=[{'frame_id': frame_id}],
            remappings=[
                ('cmd_vel', cmd_vel),
                ('reference', reference),
            ],
        ),
    ])
