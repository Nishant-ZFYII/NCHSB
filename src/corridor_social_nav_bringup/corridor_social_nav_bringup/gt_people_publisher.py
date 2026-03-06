"""
Ground-truth people publisher for Gazebo Ignition Fortress simulation.

Subscribes to Gazebo pose topics for all models via ros_gz_bridge,
filters for models whose names start with 'pedestrian_', computes
velocity via finite differencing, and publishes TrackedPeople on
/tracked_people.

Bridge setup (in launch file):
  ros2 run ros_gz_bridge parameter_bridge \
    /world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V

This publishes all dynamic model poses as TF transforms. We filter
for pedestrian models by name prefix.

Alternative approach: bridge individual model pose topics per pedestrian.
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from corridor_social_nav.msg import TrackedPerson, TrackedPeople


class GtPeoplePublisher(Node):
    def __init__(self):
        super().__init__('gt_people_publisher')

        self.declare_parameter('noise_std', 0.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('model_prefix', 'pedestrian_')
        self.declare_parameter('world_name', 'default')

        self.noise_std = self.get_parameter('noise_std').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.model_prefix = self.get_parameter('model_prefix').value
        self.world_name = self.get_parameter('world_name').value

        self.pub = self.create_publisher(TrackedPeople, '/tracked_people', 10)

        # State tracking for velocity computation
        self.prev_positions = {}   # id -> (x, y, z)
        self.prev_timestamps = {}  # id -> float (sec)
        self.person_ids = {}       # model_name -> uint32 id

        self.next_id = 1

        # Subscribe to Gazebo dynamic pose info (all models)
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/{}/dynamic_pose/info'.format(self.world_name),
            self.pose_callback,
            10
        )

        # Also try the pose/info topic (Fortress uses different topic names)
        self.pose_sub_alt = self.create_subscription(
            TFMessage,
            '/world/{}/pose/info'.format(self.world_name),
            self.pose_callback,
            10
        )

        # Publish at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        # Buffer: latest poses received this cycle
        self.current_poses = {}  # model_name -> TransformStamped

        self.get_logger().info(
            f'GT People Publisher: prefix={self.model_prefix}, '
            f'rate={self.publish_rate}Hz, noise_std={self.noise_std}')

    def pose_callback(self, msg: TFMessage):
        for tf in msg.transforms:
            name = tf.child_frame_id
            if name.startswith(self.model_prefix):
                self.current_poses[name] = tf

    def publish(self):
        if not self.current_poses:
            return

        msg = TrackedPeople()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        now_sec = self.get_clock().now().nanoseconds / 1e9

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

            # Velocity via finite differencing
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
