"""
Ground-truth collision detector for simulation experiments.

Uses GT robot pose and GT pedestrian poses to compute:
  - Minimum human-robot distance
  - Collision events (dist < robot_radius + person_radius)
  - Time-to-collision (TTC) from relative positions and velocities

Publishes metrics consumed by metrics_logger.py.

Does NOT use LiDAR scans (scan-based detection misses actors between
rays -- Phase 0 finding). Uses GT poses for accuracy.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

from corridor_social_nav.msg import TrackedPeople


class GtCollisionDetector(Node):
    def __init__(self):
        super().__init__('gt_collision_detector')

        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('person_radius', 0.25)
        self.declare_parameter('ttc_horizon', 5.0)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.person_radius = self.get_parameter('person_radius').value
        self.ttc_horizon = self.get_parameter('ttc_horizon').value
        self.collision_threshold = self.robot_radius + self.person_radius

        # Robot state
        self.robot_x = None
        self.robot_y = None
        self.robot_vx = 0.0
        self.robot_vy = 0.0

        # Subscribers
        self.gt_pose_sub = self.create_subscription(
            Pose, '/ground_truth_pose', self.robot_pose_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.people_sub = self.create_subscription(
            TrackedPeople, '/tracked_people', self.people_cb, 10)

        # Publishers
        self.min_dist_pub = self.create_publisher(
            Float64, '/min_human_distance', 10)
        self.collision_pub = self.create_publisher(
            Bool, '/collision_event', 10)
        self.ttc_pub = self.create_publisher(
            Float64, '/gt_ttc', 10)

        # Collision counter for logging
        self.collision_count = 0
        self.prev_collision = False

        self.get_logger().info(
            f'GT Collision Detector: robot_r={self.robot_radius}, '
            f'person_r={self.person_radius}, '
            f'threshold={self.collision_threshold:.3f}m')

    def robot_pose_cb(self, msg: Pose):
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y

    def odom_cb(self, msg: Odometry):
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_vy = msg.twist.twist.linear.y
        if self.robot_x is None:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y

    def people_cb(self, msg: TrackedPeople):
        if self.robot_x is None:
            return

        min_dist = float('inf')
        min_ttc = self.ttc_horizon
        is_collision = False

        for person in msg.people:
            dx = person.position.x - self.robot_x
            dy = person.position.y - self.robot_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < min_dist:
                min_dist = dist

            if dist < self.collision_threshold:
                is_collision = True

            # TTC: time for distance to reach collision threshold
            # relative velocity along the connecting line
            rel_vx = self.robot_vx - person.velocity.x
            rel_vy = self.robot_vy - person.velocity.y

            # Project relative velocity onto the line connecting robot-person
            if dist > 1e-6:
                # closing speed (positive = approaching)
                closing_speed = (dx * rel_vx + dy * rel_vy) / dist
                if closing_speed > 0.01:
                    clearance = dist - self.collision_threshold
                    if clearance > 0:
                        ttc = clearance / closing_speed
                        if ttc < min_ttc:
                            min_ttc = ttc
                    else:
                        min_ttc = 0.0

        # Publish min distance
        dist_msg = Float64()
        dist_msg.data = min_dist
        self.min_dist_pub.publish(dist_msg)

        # Publish collision event (rising-edge counting)
        col_msg = Bool()
        col_msg.data = is_collision
        self.collision_pub.publish(col_msg)

        if is_collision and not self.prev_collision:
            self.collision_count += 1
            self.get_logger().warn(
                f'COLLISION #{self.collision_count} detected! '
                f'min_dist={min_dist:.3f}m')
        self.prev_collision = is_collision

        # Publish TTC
        ttc_msg = Float64()
        ttc_msg.data = min_ttc
        self.ttc_pub.publish(ttc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GtCollisionDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException,
            RuntimeError):
        pass
    finally:
        try:
            node.get_logger().info(
                f'Total collisions detected: {node.collision_count}')
        except Exception:
            pass
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
