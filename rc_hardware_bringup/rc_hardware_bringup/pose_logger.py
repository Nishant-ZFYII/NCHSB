#!/usr/bin/env python3
"""
Record the map→base_link transform from SLAM Toolbox at a fixed rate
and write it to a TUM-format file for comparison with ground truth.

TUM format: timestamp tx ty tz qx qy qz qw

Usage:
    ros2 run rc_hardware_bringup pose_logger.py --ros-args \
        -p output_file:=/path/to/estimated_poses.txt \
        -p rate:=20.0
"""

import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        self.declare_parameter('output_file', '/tmp/estimated_poses.txt')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('target_frame', 'base_link')

        self._output_file = self.get_parameter('output_file').value
        rate = self.get_parameter('rate').value
        self._source = self.get_parameter('source_frame').value
        self._target = self.get_parameter('target_frame').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._file = open(self._output_file, 'w')
        self._count = 0

        self._timer = self.create_timer(1.0 / rate, self._log_pose)
        self.get_logger().info(
            f'PoseLogger: {self._source}→{self._target} @ {rate} Hz '
            f'→ {self._output_file}')

    def _log_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._source, self._target, rclpy.time.Time())
        except Exception:
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

        self._file.write(
            f'{stamp:.9f} {t.x:.6f} {t.y:.6f} {t.z:.6f} '
            f'{r.x:.6f} {r.y:.6f} {r.z:.6f} {r.w:.6f}\n')
        self._count += 1

        if self._count % 200 == 0:
            self._file.flush()
            self.get_logger().info(f'Logged {self._count} poses')

    def destroy_node(self):
        self._file.flush()
        self._file.close()
        self.get_logger().info(
            f'PoseLogger: wrote {self._count} poses to {self._output_file}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
