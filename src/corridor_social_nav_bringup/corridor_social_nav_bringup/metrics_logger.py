"""
Per-trial metrics logger for corridor social navigation experiments.

Subscribes to experiment monitoring topics, accumulates metrics in real-time,
and writes a single CSV row when the trial ends (signaled by /trial_result).

Design:
  - All metrics are accumulated incrementally (no large buffers)
  - Writes CSV with header if file doesn't exist, appends row otherwise
  - Exits cleanly after writing results
  - Publishes nothing (pure observer)
"""

import csv
import os
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, String


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('output_file', 'trial_metrics.csv')
        self.declare_parameter('scenario', 'unknown')
        self.declare_parameter('controller', 'unknown')
        self.declare_parameter('seed', 0)
        self.declare_parameter('timeout_s', 120.0)

        self.output_file = self.get_parameter('output_file').value
        self.scenario = self.get_parameter('scenario').value
        self.controller = self.get_parameter('controller').value
        self.seed = self.get_parameter('seed').value
        self.timeout_s = self.get_parameter('timeout_s').value

        # Metric accumulators
        self.collision_count = 0
        self.prev_collision = False
        self.min_separation = float('inf')
        self.ttc_violations = 0
        self.shield_interventions = 0
        self.shield_total_samples = 0
        self.path_length = 0.0
        self.stuck_time = 0.0
        self.prev_odom_x = None
        self.prev_odom_y = None
        self.prev_odom_time = None
        self.success = False
        self.time_to_goal = 0.0
        self.trial_ended = False

        # Subscriptions
        self.create_subscription(Bool, '/collision_event', self.collision_cb, 10)
        self.create_subscription(Float64, '/min_human_distance', self.min_dist_cb, 10)
        self.create_subscription(Float64, '/gt_ttc', self.ttc_cb, 10)
        self.create_subscription(Bool, '/cbf/intervention', self.shield_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/trial_result', self.result_cb, 10)

        self.get_logger().info(
            f'MetricsLogger: scenario={self.scenario}, controller={self.controller}, '
            f'seed={self.seed}, output={self.output_file}')

    def collision_cb(self, msg):
        if msg.data and not self.prev_collision:
            self.collision_count += 1
        self.prev_collision = msg.data

    def min_dist_cb(self, msg):
        if msg.data < self.min_separation:
            self.min_separation = msg.data

    def ttc_cb(self, msg):
        if msg.data < 1.0:
            self.ttc_violations += 1

    def shield_cb(self, msg):
        self.shield_total_samples += 1
        if msg.data:
            self.shield_interventions += 1

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = self.get_clock().now().nanoseconds / 1e9
        v = msg.twist.twist.linear.x

        if self.prev_odom_x is not None:
            dx = x - self.prev_odom_x
            dy = y - self.prev_odom_y
            self.path_length += math.sqrt(dx * dx + dy * dy)

            dt = now - self.prev_odom_time
            if dt > 0 and abs(v) < 0.02:
                self.stuck_time += dt

        self.prev_odom_x = x
        self.prev_odom_y = y
        self.prev_odom_time = now

    def result_cb(self, msg):
        if self.trial_ended:
            return
        self.trial_ended = True

        result_str = msg.data
        if result_str.startswith('SUCCESS'):
            self.success = True
            parts = result_str.split(':')
            if len(parts) > 1:
                self.time_to_goal = float(parts[1])
        elif ':' in result_str:
            parts = result_str.split(':')
            self.time_to_goal = float(parts[1])

        self.write_csv()
        self.get_logger().info('Trial ended, metrics written. Shutting down.')
        self.create_timer(1.0, lambda: rclpy.shutdown())

    def write_csv(self):
        shield_pct = 0.0
        if self.shield_total_samples > 0:
            shield_pct = 100.0 * self.shield_interventions / self.shield_total_samples

        if self.min_separation == float('inf'):
            self.min_separation = -1.0  # no pedestrians detected

        row = {
            'scenario': self.scenario,
            'controller': self.controller,
            'seed': self.seed,
            'success': int(self.success),
            'collision_count': self.collision_count,
            'min_separation_m': round(self.min_separation, 4),
            'ttc_violations': self.ttc_violations,
            'shield_interventions_pct': round(shield_pct, 2),
            'time_to_goal_s': round(self.time_to_goal, 2),
            'path_length_m': round(self.path_length, 3),
            'stuck_time_s': round(self.stuck_time, 2),
        }

        file_exists = os.path.exists(self.output_file)
        with open(self.output_file, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

        self.get_logger().info(f'Metrics written to {self.output_file}')
        for k, v in row.items():
            self.get_logger().info(f'  {k}: {v}')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if not node.trial_ended:
            node.get_logger().warn('Exiting without trial result - writing partial metrics')
            node.write_csv()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
