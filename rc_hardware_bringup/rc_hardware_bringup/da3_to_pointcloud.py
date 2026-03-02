#!/usr/bin/env python3
"""
Convert DA3 monocular depth to PointCloud2 for Nav2 obstacle layer.

Subscribes to a DA3 depth image (float32, metres) and camera info,
back-projects valid pixels to 3D, filters by height band, and publishes
a PointCloud2 message that Nav2's ObstacleLayer can consume directly.

The height filter keeps only points between min_height and max_height
above the ground plane, matching the offline costmap_builder.py logic.
"""

import numpy as np
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge


class Da3ToPointCloud(Node):
    def __init__(self):
        super().__init__('da3_to_pointcloud')

        self.declare_parameter('depth_topic', '/depth_anything_3/depth')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('output_topic', '/da3/pointcloud')
        self.declare_parameter('output_frame', 'camera_color_optical_frame')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 0.50)
        self.declare_parameter('camera_height', 0.25)
        self.declare_parameter('downsample', 4)

        self._min_depth = self.get_parameter('min_depth').value
        self._max_depth = self.get_parameter('max_depth').value
        self._min_height = self.get_parameter('min_height').value
        self._max_height = self.get_parameter('max_height').value
        self._cam_height = self.get_parameter('camera_height').value
        self._downsample = self.get_parameter('downsample').value
        output_frame = self.get_parameter('output_frame').value

        self._bridge = CvBridge()
        self._intrinsics = None
        self._output_frame = output_frame

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, 10)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, qos_sensor)
        self._pc_pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f'DA3->PointCloud2 node started: {depth_topic} -> {output_topic}')

    def _on_camera_info(self, msg: CameraInfo):
        if self._intrinsics is None:
            self._intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
            }
            self.get_logger().info(
                f'Camera intrinsics: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}')

    def _on_depth(self, msg: Image):
        if self._intrinsics is None:
            return

        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32)

        if depth.max() > 100.0:
            depth = depth / 1000.0

        ds = self._downsample
        if ds > 1:
            depth = depth[::ds, ::ds]

        fx = self._intrinsics['fx'] / ds
        fy = self._intrinsics['fy'] / ds
        cx = self._intrinsics['cx'] / ds
        cy = self._intrinsics['cy'] / ds

        valid = (depth >= self._min_depth) & (depth <= self._max_depth)
        vs, us = np.where(valid)
        ds_vals = depth[vs, us].astype(np.float64)

        if len(ds_vals) == 0:
            return

        X = ((us.astype(np.float64) - cx) * ds_vals / fx).astype(np.float32)
        Y = ((vs.astype(np.float64) - cy) * ds_vals / fy).astype(np.float32)
        Z = ds_vals.astype(np.float32)

        # Height filter: keep obstacles in [min_height, max_height] above ground
        # In camera optical frame, Y points down, so ground is at Y = +camera_height
        height_above_ground = self._cam_height - Y
        h_mask = (
            (height_above_ground >= self._min_height) &
            (height_above_ground <= self._max_height)
        )
        X, Y, Z = X[h_mask], Y[h_mask], Z[h_mask]

        if len(X) == 0:
            return

        pc_msg = self._create_pointcloud2(msg.header, X, Y, Z)
        self._pc_pub.publish(pc_msg)

    def _create_pointcloud2(
        self, header: Header, x: np.ndarray, y: np.ndarray, z: np.ndarray,
    ) -> PointCloud2:
        """Build a PointCloud2 message from XYZ arrays."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self._output_frame

        n_points = len(x)
        msg.height = 1
        msg.width = n_points
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * n_points

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        points = np.column_stack([x, y, z]).astype(np.float32)
        msg.data = points.tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = Da3ToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
