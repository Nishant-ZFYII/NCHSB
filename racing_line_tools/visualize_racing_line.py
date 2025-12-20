#!/usr/bin/env python3
"""
ROS 2 node to visualize the full racing line as markers in RViz.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import yaml
import numpy as np


class RacingLineVisualizer(Node):
    def __init__(self):
        super().__init__('racing_line_visualizer')
        
        self.publisher_ = self.create_publisher(MarkerArray, 'racing_line_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        # Load racing line
        self.racing_line_file = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/final_racing_line.yaml'
        self.waypoints = self.load_racing_line()
        
        if self.waypoints:
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        else:
            self.get_logger().error("Failed to load racing line.")

    def load_racing_line(self):
        try:
            with open(self.racing_line_file, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Error loading racing line: {e}")
            return []

    def publish_markers(self):
        if not self.waypoints:
            return

        marker_array = MarkerArray()
        
        # Path marker (line strip) - ORANGE
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "racing_line_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.08  # Line width
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 0.5
        path_marker.color.b = 0.0  # Orange

        for wp in self.waypoints:
            p = Point()
            p.x = wp['x']
            p.y = wp['y']
            p.z = 0.05
            path_marker.points.append(p)
        
        # Close the loop
        if len(path_marker.points) > 1:
            path_marker.points.append(path_marker.points[0])

        marker_array.markers.append(path_marker)

        # Start marker (green sphere)
        if self.waypoints:
            start = self.waypoints[0]
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = "racing_line_start"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = start['x']
            start_marker.pose.position.y = start['y']
            start_marker.pose.position.z = 0.1
            start_marker.scale.x = 0.15
            start_marker.scale.y = 0.15
            start_marker.scale.z = 0.15
            start_marker.color.a = 1.0
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0  # Green
            marker_array.markers.append(start_marker)

        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RacingLineVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

