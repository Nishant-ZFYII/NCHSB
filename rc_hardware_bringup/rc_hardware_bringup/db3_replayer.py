#!/usr/bin/env python3
"""
Stream-replay a ROS2 sqlite3 (.db3) rosbag on ROS2 topics.

Same wall-clock restamping strategy as mcap_replayer.py but reads
the sqlite3 storage format used by ROS2 Humble's default recorder.

The bag_path parameter should point to the .db3 file directly
(e.g. /path/to/bag_0.db3).

Usage:
    ros2 run rc_hardware_bringup db3_replayer.py --ros-args \
        -p bag_path:=/path/to/bag_0.db3 -p rate:=1.0 -p loop:=true
"""

import sqlite3 as sql3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time as TimeMsg

from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock as ClockMsg

TYPE_MAP = {
    'sensor_msgs/msg/Image': Image,
    'sensor_msgs/msg/CameraInfo': CameraInfo,
    'sensor_msgs/msg/Imu': Imu,
    'sensor_msgs/msg/LaserScan': LaserScan,
    'tf2_msgs/msg/TFMessage': TFMessage,
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
    elif hasattr(ros_msg, 'transforms'):
        for tf in ros_msg.transforms:
            tf.header.stamp = stamp
    return ros_msg


class Db3Replayer(Node):
    def __init__(self):
        super().__init__('db3_replayer')

        self.declare_parameter('bag_path', '')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', True)

        bag_path = self.get_parameter('bag_path').value
        self._rate = self.get_parameter('rate').value
        self._loop = self.get_parameter('loop').value

        if not bag_path:
            self.get_logger().error('bag_path parameter is required')
            raise SystemExit(1)

        self._bag_path = bag_path
        self._topic_pubs = {}
        self._topic_classes = {}
        self._topic_id_to_name = {}
        self._running = True

        self._clock_pub = self.create_publisher(ClockMsg, '/clock', 10)

        self.get_logger().info(f'Replaying {bag_path} at {self._rate}x')
        self._discover_topics()

        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()

    def _discover_topics(self):
        conn = sql3.connect(self._bag_path)
        cursor = conn.cursor()
        cursor.execute('SELECT id, name, type FROM topics')
        for tid, name, msg_type in cursor.fetchall():
            if msg_type in TYPE_MAP:
                self._topic_id_to_name[tid] = name
                self._topic_classes[name] = TYPE_MAP[msg_type]
        conn.close()

        for topic, msg_class in self._topic_classes.items():
            qos = QoSProfile(depth=10)
            if topic == '/tf_static':
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._topic_pubs[topic] = self.create_publisher(
                msg_class, topic, qos)

        self.get_logger().info(
            f'Found {len(self._topic_pubs)} topics: '
            f'{list(self._topic_pubs.keys())}')

    def _replay_loop(self):
        while self._running:
            start_wall = None
            first_bag_time = None
            msg_count = 0
            last_bag_time = 0

            self.get_logger().info('Playback started')

            try:
                conn = sql3.connect(self._bag_path)
                cursor = conn.cursor()
                cursor.execute(
                    'SELECT topic_id, timestamp, data '
                    'FROM messages ORDER BY timestamp')

                for topic_id, timestamp_ns, data in cursor:
                    if not self._running:
                        conn.close()
                        return

                    topic = self._topic_id_to_name.get(topic_id)
                    if topic is None or topic not in self._topic_pubs:
                        continue

                    last_bag_time = timestamp_ns

                    if first_bag_time is None:
                        first_bag_time = timestamp_ns
                        start_wall = time.monotonic()

                    bag_elapsed_s = (timestamp_ns - first_bag_time) / 1e9
                    target_wall = start_wall + bag_elapsed_s / self._rate

                    now = time.monotonic()
                    if target_wall > now:
                        time.sleep(target_wall - now)

                    msg_class = self._topic_classes[topic]
                    ros_msg = deserialize_message(bytes(data), msg_class)

                    stamp = _wall_stamp()
                    _restamp(ros_msg, stamp)
                    self._topic_pubs[topic].publish(ros_msg)
                    msg_count += 1

                    clock_msg = ClockMsg()
                    clock_msg.clock = stamp
                    self._clock_pub.publish(clock_msg)

                    if msg_count % 5000 == 0:
                        self.get_logger().info(
                            f'Published {msg_count} msgs, '
                            f't={bag_elapsed_s:.1f}s')

                conn.close()

            except Exception as e:
                self.get_logger().error(f'Replay error: {e}')
                return

            total_s = ((last_bag_time - first_bag_time) / 1e9
                       if first_bag_time else 0)
            self.get_logger().info(
                f'Pass complete: {msg_count} msgs over {total_s:.1f}s')

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
    node = Db3Replayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
