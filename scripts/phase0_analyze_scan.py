#!/usr/bin/env python3
"""
Phase 0: Automated LiDAR scan analyzer for the actor visibility spike test.

Subscribes to /scan, collects a few messages, and checks if there are
range returns at the known angles toward the test cylinders.

Run AFTER the spike test world is launched:
  cd ~/MS_Project/NCHSB
  source /opt/ros/humble/setup.bash
  python3 scripts/phase0_analyze_scan.py

Test geometry (robot at 1.0, 0.0 facing +X):
  RED static cylinder:   (3.0, 0.0) => 2.0m ahead,  bearing ~0 deg
  BLUE dynamic cylinder: (1.0, 2.0) => 2.0m to left, bearing ~90 deg
  South wall at y=-2 => 2.0m to right, bearing ~-90 deg
  North wall at y=+2 => 2.0m to left,  bearing ~+90 deg
"""

import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('phase0_scan_analyzer')
        self.scans_collected = []
        self.max_scans = 10

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.get_logger().info('Waiting for /scan messages...')
        self.get_logger().info('(Gazebo + bridge must be running first)')

    def scan_cb(self, msg: LaserScan):
        self.scans_collected.append(msg)
        n = len(self.scans_collected)
        self.get_logger().info(f'Collected scan {n}/{self.max_scans}')

        if n >= self.max_scans:
            self.analyze()
            rclpy.shutdown()

    def analyze(self):
        print('\n' + '=' * 65)
        print('   PHASE 0: LiDAR-Actor Visibility Spike — RESULTS')
        print('=' * 65)

        msg = self.scans_collected[-1]
        ranges = np.array(msg.ranges)
        n_rays = len(ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        angles = np.array([angle_min + i * angle_inc for i in range(n_rays)])

        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        n_valid = int(np.sum(valid_mask))

        print(f'\n  Scan stats:')
        print(f'    Total rays:   {n_rays}')
        print(f'    Valid returns: {n_valid} ({100*n_valid/n_rays:.1f}%)')
        print(f'    Inf/NaN:      {n_rays - n_valid}')
        print(f'    Range limits:  [{msg.range_min:.2f}, {msg.range_max:.2f}] m')
        print(f'    Angle range:  [{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}] deg')

        checks = [
            ('RED cylinder (static, 2m ahead)',  0.0,   1.5, 2.5),
            ('BLUE cylinder (dynamic, 2m left)', math.pi/2,  1.5, 2.5),
            ('South wall (2m right)',           -math.pi/2,  1.5, 2.5),
            ('North wall (2m left)',             math.pi/2,  1.5, 2.5),
        ]

        print(f'\n  --- Directional Checks (±15° cone) ---')
        results = {}

        for label, center_angle, min_r, max_r in checks:
            half_cone = math.radians(15)
            lo = center_angle - half_cone
            hi = center_angle + half_cone

            if lo < -math.pi:
                mask_angle = (angles > lo + 2*math.pi) | (angles < hi)
            elif hi > math.pi:
                mask_angle = (angles > lo) | (angles < hi - 2*math.pi)
            else:
                mask_angle = (angles > lo) & (angles < hi)

            cone_ranges = ranges[mask_angle]
            cone_valid = np.isfinite(cone_ranges) & (cone_ranges > msg.range_min) & (cone_ranges < msg.range_max)

            n_cone = int(np.sum(mask_angle))
            n_cone_valid = int(np.sum(cone_valid))

            if n_cone_valid > 0:
                valid_ranges = cone_ranges[cone_valid]
                r_min = float(np.min(valid_ranges))
                r_mean = float(np.mean(valid_ranges))
                in_expected = (r_min >= min_r * 0.5) and (r_min <= max_r * 1.5)
                status = 'VISIBLE' if in_expected else f'RANGE MISMATCH ({r_min:.2f}m)'
            else:
                r_min = float('inf')
                r_mean = float('inf')
                status = 'INVISIBLE'

            icon = '[OK]  ' if status == 'VISIBLE' else '[FAIL]'
            results[label] = status

            print(f'\n    {icon} {label}')
            print(f'           Bearing:  {math.degrees(center_angle):.0f}°')
            print(f'           Rays:     {n_cone} in cone, {n_cone_valid} valid')
            print(f'           Min dist: {r_min:.3f} m')
            if n_cone_valid > 0:
                print(f'           Mean:     {r_mean:.3f} m')
            print(f'           Expected: [{min_r:.1f} – {max_r:.1f}] m')
            print(f'           Status:   {status}')

        # Overall verdict
        print('\n' + '-' * 65)
        print('   VERDICT')
        print('-' * 65)

        static_ok = results.get('RED cylinder (static, 2m ahead)', '') == 'VISIBLE'
        dynamic_ok = results.get('BLUE cylinder (dynamic, 2m left)', '') == 'VISIBLE'

        if static_ok and dynamic_ok:
            print('   PASS: Both static and dynamic cylinders visible to gpu_lidar!')
            print('   -> Pedestrian cylinder models will work for all experiments.')
            print('   -> Use static=false models driven by Gazebo pose service.')
        elif static_ok:
            print('   PARTIAL: Static cylinder visible, dynamic cylinder NOT visible.')
            print('   -> Use static models repositioned via Gazebo service, OR')
            print('   -> Inject virtual scan points from GT poses.')
        else:
            print('   FAIL: Cylinders NOT visible to gpu_lidar.')
            print('   -> FALLBACK REQUIRED: GT-pose injection into costmap,')
            print('   ->   bypassing LiDAR entirely for pedestrian detection.')

        print('=' * 65 + '\n')


def main():
    rclpy.init()
    node = ScanAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
