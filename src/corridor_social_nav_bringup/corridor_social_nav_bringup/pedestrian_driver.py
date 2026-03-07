"""
Pedestrian driver node for corridor social navigation experiments.

Drives spawned pedestrian cylinder models along waypoint trajectories
by calling the Ignition Fortress /world/<world>/set_pose service
at a fixed rate.

Each pedestrian moves in straight-line segments between waypoints
at its configured speed. After reaching the final waypoint, the
pedestrian holds position (no looping).

Pedestrian configurations are passed as a JSON-encoded ROS parameter
containing a list of {id, speed, start_delay, waypoints} dicts.
"""

import json
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class PedestrianState:
    def __init__(self, ped_id: int, speed: float, start_delay: float,
                 waypoints: list, spawn_z: float = 0.85):
        self.ped_id = ped_id
        self.name = f'pedestrian_{ped_id}'
        self.speed = speed
        self.start_delay = start_delay
        self.waypoints = waypoints
        self.spawn_z = spawn_z

        self.current_wp_idx = 0
        self.x = waypoints[0][0]
        self.y = waypoints[0][1]
        self.started = False
        self.finished = False
        self.start_time = None


class PedestrianDriver(Node):
    def __init__(self):
        super().__init__('pedestrian_driver')

        self.declare_parameter('pedestrians_json', '[]')
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('world_name', 'default')

        json_str = self.get_parameter('pedestrians_json').value
        rate = self.get_parameter('update_rate').value
        self.world_name = self.get_parameter('world_name').value

        ped_configs = json.loads(json_str)
        self.pedestrians = []
        for cfg in ped_configs:
            self.pedestrians.append(PedestrianState(
                ped_id=cfg['id'],
                speed=cfg['speed'],
                start_delay=cfg['start_delay'],
                waypoints=cfg['waypoints'],
                spawn_z=cfg.get('spawn_z', 0.85),
            ))

        self._pending_procs = {}

        self.trial_active = False
        self.sim_start_time = None

        self.create_subscription(
            Empty, '/trial_active', self._on_trial_active, 10)

        self.timer = self.create_timer(1.0 / rate, self.update)

        self.get_logger().info(
            f'PedestrianDriver: {len(self.pedestrians)} pedestrians, '
            f'rate={rate}Hz, world={self.world_name}, '
            f'waiting for /trial_active signal')

    def _on_trial_active(self, msg):
        if not self.trial_active:
            self.trial_active = True
            self.sim_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                'Received /trial_active -- pedestrians will begin moving')

    def update(self):
        if not self.trial_active:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.sim_start_time
        dt = 1.0 / 20.0  # approximate

        for ped in self.pedestrians:
            if ped.finished:
                continue

            if not ped.started:
                if elapsed >= ped.start_delay:
                    ped.started = True
                    ped.start_time = now
                    self.get_logger().info(
                        f'{ped.name} started moving (delay={ped.start_delay:.1f}s)')
                else:
                    continue

            if ped.current_wp_idx >= len(ped.waypoints) - 1:
                ped.finished = True
                self.get_logger().info(f'{ped.name} reached final waypoint')
                continue

            target = ped.waypoints[ped.current_wp_idx + 1]
            dx = target[0] - ped.x
            dy = target[1] - ped.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.05:
                ped.current_wp_idx += 1
                if ped.current_wp_idx >= len(ped.waypoints) - 1:
                    ped.finished = True
                    self.get_logger().info(f'{ped.name} reached final waypoint')
                continue

            step = ped.speed * dt
            if step > dist:
                step = dist

            ped.x += (dx / dist) * step
            ped.y += (dy / dist) * step

            self._set_pose(ped.name, ped.x, ped.y, ped.spawn_z)

    def _set_pose(self, name: str, x: float, y: float, z: float):
        prev = self._pending_procs.get(name)
        if prev is not None and prev.poll() is None:
            return

        req = (f'name: "{name}", '
               f'position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{w: 1.0}}')
        self._pending_procs[name] = subprocess.Popen(
            ['ign', 'service',
             '-s', f'/world/{self.world_name}/set_pose',
             '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '200',
             '--req', req],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianDriver()
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
