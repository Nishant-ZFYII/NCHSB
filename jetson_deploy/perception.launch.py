"""
Launch file for the perception stack on Jetson Orin Nano 8GB.

Launches:
  1. DA3-Small depth estimation (via GerdsenAI wrapper)
  2. YOLOv8-nano object detection

Prerequisites:
  - DA3 wrapper cloned at ~/da3_ros2 and built (./run.sh once)
  - YOLOv8-nano TensorRT engine exported at ~/yolov8n.engine
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/camera/color/image_raw",
            description="RGB image topic from camera",
        ),
        DeclareLaunchArgument(
            "yolo_engine",
            default_value="~/yolov8n.engine",
            description="Path to YOLOv8 TensorRT engine",
        ),
        DeclareLaunchArgument(
            "yolo_confidence",
            default_value="0.4",
            description="YOLO confidence threshold",
        ),
        DeclareLaunchArgument(
            "yolo_every_n",
            default_value="3",
            description="Run YOLO every N frames (saves GPU for DA3)",
        ),

        LogInfo(msg="Starting perception stack (DA3 + YOLO)"),
        LogInfo(msg="NOTE: DA3 must be started separately via ~/da3_ros2/run.sh"),

        Node(
            package="jetson_deploy",
            executable="yolo_node",
            name="yolo_node",
            parameters=[{
                "engine_path": LaunchConfiguration("yolo_engine"),
                "confidence_threshold": LaunchConfiguration("yolo_confidence"),
                "image_topic": LaunchConfiguration("camera_topic"),
                "run_every_n": LaunchConfiguration("yolo_every_n"),
            }],
            output="screen",
        ),
    ])
