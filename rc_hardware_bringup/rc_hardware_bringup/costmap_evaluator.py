#!/usr/bin/env python3
"""
Record local costmap statistics and save periodic snapshots for
ablation comparison across different sensor configurations.

Subscribes to the local costmap and computes per-frame metrics:
  - Occupied cell count and ratio
  - Free cell count and ratio
  - Unknown cell count
  - Obstacle centroid positions

Saves:
  - CSV log of per-frame statistics
  - PNG snapshots of the costmap at configurable intervals
  - Summary JSON at shutdown

Usage:
  ros2 run rc_hardware_bringup costmap_evaluator.py --ros-args \
      -p config_label:=L+D \
      -p output_dir:=/home/nishant/maps/ablation_results \
      -p snapshot_interval:=5.0
"""

import os
import csv
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


LETHAL_COST = 100
FREE_THRESHOLD = 1
UNKNOWN_VALUE = -1


class CostmapEvaluator(Node):
    def __init__(self):
        super().__init__('costmap_evaluator')

        self.declare_parameter('config_label', 'L')
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/maps/ablation_results'))
        self.declare_parameter('snapshot_interval', 5.0)
        self.declare_parameter('costmap_topic',
                               '/local_costmap/costmap')

        self._label = self.get_parameter('config_label').value
        self._output_dir = self.get_parameter('output_dir').value
        self._snap_interval = self.get_parameter('snapshot_interval').value
        topic = self.get_parameter('costmap_topic').value

        self._run_dir = os.path.join(self._output_dir, self._label)
        os.makedirs(self._run_dir, exist_ok=True)

        csv_path = os.path.join(self._run_dir, 'costmap_stats.csv')
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp', 'frame_idx', 'width', 'height',
            'occupied_cells', 'free_cells', 'unknown_cells',
            'occupied_ratio', 'obstacle_centroid_x', 'obstacle_centroid_y',
        ])

        self._frame_idx = 0
        self._last_snap_time = 0.0
        self._stats_summary = {
            'config': self._label,
            'total_frames': 0,
            'occupied_cells_mean': 0.0,
            'occupied_cells_std': 0.0,
        }
        self._occupied_counts = []

        self._sub = self.create_subscription(
            OccupancyGrid, topic, self._on_costmap, 10)

        self.get_logger().info(
            f'CostmapEvaluator started: config={self._label}, '
            f'output={self._run_dir}')

    def _on_costmap(self, msg: OccupancyGrid):
        now = time.time()
        data = np.array(msg.data, dtype=np.int8)
        w, h = msg.info.width, msg.info.height
        grid = data.reshape((h, w))

        occupied = np.sum(grid >= LETHAL_COST)
        free = np.sum((grid >= 0) & (grid < FREE_THRESHOLD))
        unknown = np.sum(grid == UNKNOWN_VALUE)
        total = w * h
        occ_ratio = float(occupied) / total if total > 0 else 0.0

        obs_ys, obs_xs = np.where(grid >= LETHAL_COST)
        if len(obs_xs) > 0:
            res = msg.info.resolution
            cx = msg.info.origin.position.x + float(obs_xs.mean()) * res
            cy = msg.info.origin.position.y + float(obs_ys.mean()) * res
        else:
            cx, cy = float('nan'), float('nan')

        self._csv_writer.writerow([
            f'{now:.3f}', self._frame_idx, w, h,
            int(occupied), int(free), int(unknown),
            f'{occ_ratio:.6f}', f'{cx:.4f}', f'{cy:.4f}',
        ])

        self._occupied_counts.append(int(occupied))

        if (now - self._last_snap_time) >= self._snap_interval:
            self._save_snapshot(grid, w, h)
            self._last_snap_time = now

        self._frame_idx += 1

    def _save_snapshot(self, grid: np.ndarray, w: int, h: int):
        try:
            import cv2
        except ImportError:
            return

        vis = np.full((h, w, 3), 128, dtype=np.uint8)
        vis[grid == 0] = [255, 255, 255]
        vis[grid >= LETHAL_COST] = [0, 0, 0]
        vis[(grid > 0) & (grid < LETHAL_COST)] = [180, 180, 255]
        vis[grid == UNKNOWN_VALUE] = [128, 128, 128]

        path = os.path.join(self._run_dir,
                            f'costmap_{self._frame_idx:05d}.png')
        cv2.imwrite(path, vis)
        self.get_logger().info(f'Snapshot saved: {path}')

    def destroy_node(self):
        if self._occupied_counts:
            arr = np.array(self._occupied_counts)
            self._stats_summary.update({
                'total_frames': len(arr),
                'occupied_cells_mean': float(arr.mean()),
                'occupied_cells_std': float(arr.std()),
                'occupied_cells_min': int(arr.min()),
                'occupied_cells_max': int(arr.max()),
            })

        summary_path = os.path.join(self._run_dir, 'summary.json')
        with open(summary_path, 'w') as f:
            json.dump(self._stats_summary, f, indent=2)
        self.get_logger().info(f'Summary saved: {summary_path}')

        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CostmapEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
