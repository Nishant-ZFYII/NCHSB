"""
Ground-truth people publisher for Gazebo Ignition Fortress simulation.

Subscribes to Gazebo pose topics for all models via ros_gz_bridge,
filters for models whose names start with 'pedestrian_', computes
velocity via finite differencing, and publishes TrackedPeople on
/tracked_people.

Robustness features:
  - Stale model pruning: entries not updated within `stale_timeout` are removed.
  - Optional trial gating: if `gate_on_trial_active` is True, no messages are
    published until /trial_active has been received (prevents omniscient
    social cost before the robot starts navigating).

Bridge setup (in launch file):
  ros2 run ros_gz_bridge parameter_bridge \
    /world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Empty

from corridor_social_nav.msg import TrackedPerson, TrackedPeople


class GtPeoplePublisher(Node):
    def __init__(self):
        super().__init__('gt_people_publisher')

        self.declare_parameter('noise_std', 0.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('model_prefix', 'pedestrian_')
        self.declare_parameter('world_name', 'default')
        self.declare_parameter('stale_timeout', 1.0)
        self.declare_parameter('gate_on_trial_active', True)

        self.noise_std = self.get_parameter('noise_std').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.model_prefix = self.get_parameter('model_prefix').value
        self.world_name = self.get_parameter('world_name').value
        self.stale_timeout = self.get_parameter('stale_timeout').value
        self.gate_on_trial_active = self.get_parameter('gate_on_trial_active').value

        self.pub = self.create_publisher(TrackedPeople, '/tracked_people', 10)

        self.prev_positions = {}   # id -> (x, y, z)
        self.prev_timestamps = {}  # id -> float (sec)
        self.person_ids = {}       # model_name -> uint32 id
        self.pose_update_times = {}  # model_name -> float (sec, last update)

        self.next_id = 1
        self.trial_active = not self.gate_on_trial_active

        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/{}/dynamic_pose/info'.format(self.world_name),
            self.pose_callback,
            10
        )

        self.pose_sub_alt = self.create_subscription(
            TFMessage,
            '/world/{}/pose/info'.format(self.world_name),
            self.pose_callback,
            10
        )

        if self.gate_on_trial_active:
            self.create_subscription(
                Empty, '/trial_active', self._on_trial_active, 10)
            self.get_logger().info(
                'GT People Publisher: waiting for /trial_active before publishing')

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        self.current_poses = {}  # model_name -> TransformStamped

        self.get_logger().info(
            f'GT People Publisher: prefix={self.model_prefix}, '
            f'rate={self.publish_rate}Hz, noise_std={self.noise_std}, '
            f'stale_timeout={self.stale_timeout}s')

    def _on_trial_active(self, msg):
        if not self.trial_active:
            self.trial_active = True
            self.get_logger().info('Received /trial_active -- will begin publishing')

    def pose_callback(self, msg: TFMessage):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for tf in msg.transforms:
            name = tf.child_frame_id
            if name.startswith(self.model_prefix):
                self.current_poses[name] = tf
                self.pose_update_times[name] = now_sec

    def publish(self):
        if not self.trial_active:
            return

        if not self.current_poses:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Prune stale models
        stale_names = [
            name for name, t in self.pose_update_times.items()
            if (now_sec - t) > self.stale_timeout
        ]
        for name in stale_names:
            self.current_poses.pop(name, None)
            self.pose_update_times.pop(name, None)
            pid = self.person_ids.get(name)
            if pid is not None:
                self.prev_positions.pop(pid, None)
                self.prev_timestamps.pop(pid, None)

        if not self.current_poses:
            return

        msg = TrackedPeople()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for name, tf in self.current_poses.items():
            if name not in self.person_ids:
                self.person_ids[name] = self.next_id
                self.next_id += 1

            pid = self.person_ids[name]

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            if self.noise_std > 0:
                x += np.random.normal(0, self.noise_std)
                y += np.random.normal(0, self.noise_std)

            vx, vy = 0.0, 0.0
            if pid in self.prev_positions and pid in self.prev_timestamps:
                dt = now_sec - self.prev_timestamps[pid]
                if dt > 1e-4:
                    px, py, _ = self.prev_positions[pid]
                    vx = (x - px) / dt
                    vy = (y - py) / dt

            self.prev_positions[pid] = (x, y, z)
            self.prev_timestamps[pid] = now_sec

            person = TrackedPerson()
            person.id = pid
            person.position.x = x
            person.position.y = y
            person.position.z = z
            person.velocity.x = vx
            person.velocity.y = vy
            person.velocity.z = 0.0
            person.confidence = 1.0

            msg.people.append(person)

        if msg.people:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GtPeoplePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
