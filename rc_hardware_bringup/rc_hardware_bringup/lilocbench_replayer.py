#!/usr/bin/env python3
"""
Replay LILocBench data on ROS2 topics.

Reads LiDAR scans, odometry, and TF from a converted MCAP bag (originally
ROS1) and publishes camera images from individual PNG files.  Remaps topic
names and frame IDs to match the NCHSB Nav2 costmap pipeline:

  /laser_scan_front/scan  -> /scan            (frame: laser)
  /dingo_velocity_controller/odom -> /odom
  camera PNG files        -> /camera/color/image_raw
                          -> /camera/depth/image_raw
                          -> /camera/color/camera_info

Usage:
    ros2 run rc_hardware_bringup lilocbench_replayer.py --ros-args \
        -p bag_path:=/path/to/dynamics_0_no_cam_ros2 \
        -p data_dir:=/path/to/lilocbench/dynamics_0_files \
        -p rate:=0.3
"""

import os
import glob
import time
import threading
import struct
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time as TimeMsg

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, LaserScan, JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock as ClockMsg

BAG_TYPE_MAP = {
    'sensor_msgs/msg/LaserScan': LaserScan,
    'sensor_msgs/msg/JointState': JointState,
    'nav_msgs/msg/Odometry': Odometry,
    'tf2_msgs/msg/TFMessage': TFMessage,
}

TOPIC_REMAP = {
    '/laser_scan_front/scan': '/scan',
    '/dingo_velocity_controller/odom': '/odom',
}

TOPICS_TO_SKIP = {
    '/laser_scan_rear/scan',
    '/dingo_velocity_controller/cmd_vel',
    '/joint_states',
    '/tf_static',
}

FRAME_REMAP = {
    'scan_front_link': 'laser',
}


def _wall_stamp() -> TimeMsg:
    t = time.time()
    msg = TimeMsg()
    msg.sec = int(t)
    msg.nanosec = int((t % 1) * 1e9)
    return msg


def _restamp(ros_msg, stamp: TimeMsg):
    if hasattr(ros_msg, 'header'):
        ros_msg.header.stamp = stamp
        fid = ros_msg.header.frame_id
        if fid in FRAME_REMAP:
            ros_msg.header.frame_id = FRAME_REMAP[fid]
    elif hasattr(ros_msg, 'transforms'):
        for tf in ros_msg.transforms:
            tf.header.stamp = stamp
    return ros_msg


class LILocBenchReplayer(Node):
    def __init__(self):
        super().__init__('lilocbench_replayer')

        self.declare_parameter('bag_path', '')
        self.declare_parameter('data_dir', '')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('camera', 'front')

        bag_path = self.get_parameter('bag_path').value
        data_dir = self.get_parameter('data_dir').value
        self._rate = self.get_parameter('rate').value
        self._loop = self.get_parameter('loop').value
        camera = self.get_parameter('camera').value

        if not bag_path:
            self.get_logger().error('bag_path parameter is required')
            raise SystemExit(1)
        if not data_dir:
            self.get_logger().error('data_dir parameter is required')
            raise SystemExit(1)

        self._bag_dir = bag_path
        self._data_dir = data_dir
        self._camera = camera
        self._running = True

        self._bag_pubs = {}
        self._bag_classes = {}

        self._clock_pub = self.create_publisher(ClockMsg, '/clock', 10)
        self._tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        cam_dir = os.path.join(data_dir, f'camera_{camera}')
        self._color_dir = os.path.join(cam_dir, 'color', 'images')
        self._depth_dir = os.path.join(cam_dir, 'depth', 'images')
        self._intrinsics_path = os.path.join(cam_dir, 'color', 'intrinsics.yaml')

        self._color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self._depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self._info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        self._camera_info_msg = None
        if os.path.isfile(self._intrinsics_path):
            self._camera_info_msg = self._load_camera_info()
        else:
            self.get_logger().warn(
                f'No intrinsics at {self._intrinsics_path}; '
                'camera images will not be published')

        self.get_logger().info(
            f'LILocBench replayer: bag={bag_path}, data={data_dir}, '
            f'camera={camera}, rate={self._rate}x')

        self._discover_bag_topics()

        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()

    def _load_camera_info(self) -> CameraInfo:
        with open(self._intrinsics_path) as f:
            intr = yaml.safe_load(f)
        msg = CameraInfo()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = intr['height']
        msg.width = intr['width']
        msg.distortion_model = intr.get('distortion_model', 'plumb_bob')
        msg.d = [float(x) for x in intr.get('distortion_coefficients', [])]
        msg.k = [float(x) for x in intr['K']]
        msg.r = [float(x) for x in intr['R']]
        msg.p = [float(x) for x in intr['P']]
        msg.binning_x = intr.get('binning_x', 0)
        msg.binning_y = intr.get('binning_y', 0)
        return msg

    def _discover_bag_topics(self):
        from mcap.reader import make_reader

        mcap_file = None
        if os.path.isdir(self._bag_dir):
            for f in os.listdir(self._bag_dir):
                if f.endswith('.mcap'):
                    mcap_file = os.path.join(self._bag_dir, f)
                    break
        elif self._bag_dir.endswith('.mcap'):
            mcap_file = self._bag_dir

        if mcap_file is None:
            self.get_logger().error(f'No .mcap file found in {self._bag_dir}')
            raise SystemExit(1)

        self._mcap_file = mcap_file

        with open(mcap_file, 'rb') as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            if summary:
                for cid, channel in summary.channels.items():
                    schema = summary.schemas.get(channel.schema_id)
                    if schema and schema.name in BAG_TYPE_MAP:
                        topic = channel.topic
                        if topic in TOPICS_TO_SKIP:
                            continue
                        self._bag_classes[topic] = BAG_TYPE_MAP[schema.name]

        for topic, msg_class in self._bag_classes.items():
            pub_topic = TOPIC_REMAP.get(topic, topic)
            qos = QoSProfile(depth=10)
            if pub_topic == '/tf_static':
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._bag_pubs[topic] = self.create_publisher(msg_class, pub_topic, qos)

        self.get_logger().info(
            f'Bag topics: {list(self._bag_pubs.keys())}')

    def _load_image_timestamps(self):
        """Return sorted list of (timestamp_ns, color_path, depth_path)."""
        if not os.path.isdir(self._color_dir):
            self.get_logger().warn(
                f'No camera directory found at {self._color_dir}; '
                'skipping camera images')
            return []

        color_files = sorted(glob.glob(os.path.join(self._color_dir, '*.png')))
        depth_files_set = {}
        if os.path.isdir(self._depth_dir):
            depth_files_set = {
                os.path.basename(f): f
                for f in glob.glob(os.path.join(self._depth_dir, '*.png'))
            }

        entries = []
        for cf in color_files:
            basename = os.path.basename(cf)
            ts_str = basename.replace('.png', '')
            parts = ts_str.split('.')
            ts_ns = int(parts[0]) * 1_000_000_000 + int(parts[1])
            df = depth_files_set.get(basename)
            entries.append((ts_ns, cf, df))

        return entries

    def _publish_odom_tf(self, odom_msg: Odometry, stamp: TimeMsg):
        """Extract pose from Odometry and publish as odom -> base_link TF."""
        tf_msg = TFMessage()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        tf_msg.transforms.append(t)
        self._tf_pub.publish(tf_msg)

    def _publish_image(self, path: str, encoding: str, frame_id: str,
                       stamp: TimeMsg, publisher):
        """Read a PNG and publish as sensor_msgs/Image."""
        import cv2
        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img is None:
            return

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = img.shape[0]
        msg.width = img.shape[1]

        if len(img.shape) == 3 and img.shape[2] == 3:
            msg.encoding = 'bgr8'
            msg.step = img.shape[1] * 3
        elif img.dtype == np.uint16:
            msg.encoding = '16UC1'
            msg.step = img.shape[1] * 2
        else:
            msg.encoding = encoding
            msg.step = img.shape[1] * (img.dtype.itemsize)

        msg.is_bigendian = False
        msg.data = img.tobytes()
        publisher.publish(msg)

    def _replay_loop(self):
        from mcap.reader import make_reader

        while self._running:
            image_entries = self._load_image_timestamps()
            self.get_logger().info(f'Loaded {len(image_entries)} camera frames')

            bag_msgs = []
            with open(self._mcap_file, 'rb') as f:
                reader = make_reader(f)
                for schema, channel, message in reader.iter_messages():
                    topic = channel.topic
                    if topic not in self._bag_pubs:
                        continue
                    bag_msgs.append((message.log_time, topic, message.data))

            timeline = []
            for ts_ns, topic, data in bag_msgs:
                timeline.append((ts_ns, 'bag', topic, data))
            for ts_ns, color_path, depth_path in image_entries:
                timeline.append((ts_ns, 'cam', color_path, depth_path))

            timeline.sort(key=lambda x: x[0])
            self.get_logger().info(
                f'Timeline: {len(timeline)} items, playback at {self._rate}x')

            start_wall = None
            first_ts = None
            msg_count = 0

            for item in timeline:
                if not self._running:
                    return

                ts_ns = item[0]
                if first_ts is None:
                    first_ts = ts_ns
                    start_wall = time.monotonic()

                elapsed_s = (ts_ns - first_ts) / 1e9
                target_wall = start_wall + elapsed_s / self._rate
                now = time.monotonic()
                if target_wall > now:
                    time.sleep(target_wall - now)

                stamp = _wall_stamp()

                try:
                    if item[1] == 'bag':
                        topic = item[2]
                        data = item[3]
                        msg_class = self._bag_classes[topic]
                        ros_msg = deserialize_message(data, msg_class)
                        _restamp(ros_msg, stamp)
                        self._bag_pubs[topic].publish(ros_msg)
                        if topic == '/dingo_velocity_controller/odom':
                            self._publish_odom_tf(ros_msg, stamp)
                    elif item[1] == 'cam':
                        color_path = item[2]
                        depth_path = item[3]
                        self._publish_image(
                            color_path, 'bgr8',
                            'camera_color_optical_frame', stamp,
                            self._color_pub)
                        if depth_path:
                            self._publish_image(
                                depth_path, '16UC1',
                                'camera_color_optical_frame', stamp,
                                self._depth_pub)
                        self._camera_info_msg.header.stamp = stamp
                        self._info_pub.publish(self._camera_info_msg)

                    clock_msg = ClockMsg()
                    clock_msg.clock = stamp
                    self._clock_pub.publish(clock_msg)
                except Exception:
                    if not self._running:
                        return
                    raise

                msg_count += 1
                if msg_count % 5000 == 0:
                    self.get_logger().info(
                        f'Published {msg_count} items, t={elapsed_s:.1f}s')

            total_s = (timeline[-1][0] - first_ts) / 1e9 if first_ts else 0
            self.get_logger().info(
                f'Pass complete: {msg_count} items over {total_s:.1f}s')

            if not self._loop:
                self.get_logger().info('Playback finished.')
                return
            self.get_logger().info('Looping...')

    def destroy_node(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LILocBenchReplayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
