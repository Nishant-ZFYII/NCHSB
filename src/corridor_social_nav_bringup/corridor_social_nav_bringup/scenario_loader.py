"""
Scenario loader for corridor social navigation experiments.

Reads scenario YAML files, applies deterministic seed-based randomization
to pedestrian parameters (speed, lateral offset, start delay), and provides
spawn commands and trajectory data for the experiment runner.

Design decisions:
  - Pure Python library, no ROS dependency (testable standalone)
  - Uses numpy.random.RandomState(seed) for reproducibility
  - Each pedestrian gets randomized: speed, lateral_offset, start_delay
  - Waypoints are adjusted by lateral_offset (perpendicular to first segment)
  - Returns structured data that the experiment runner uses to spawn peds
    and drive their trajectories via a separate ROS node
"""

import yaml
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
from pathlib import Path


@dataclass
class PedestrianInstance:
    """A fully resolved pedestrian for a specific seed."""

    id: int
    speed: float
    start_delay: float
    waypoints: List[Tuple[float, float]]  # already offset-adjusted
    spawn_x: float
    spawn_y: float
    spawn_z: float  # always 0.85 (half of 1.7m cylinder height)


@dataclass
class TrialConfig:
    """Complete configuration for a single trial."""

    scenario_name: str
    world_file: str
    robot_start_x: float
    robot_start_y: float
    robot_start_yaw: float
    goal_x: float
    goal_y: float
    timeout: float
    seed: int
    pedestrians: List[PedestrianInstance]


class ScenarioLoader:
    def __init__(self, scenario_path: str):
        """Load and parse a scenario YAML file."""
        with open(scenario_path, "r") as f:
            raw = yaml.safe_load(f)
        self._data = raw["scenario"]
        self._path = scenario_path

    @property
    def name(self) -> str:
        return self._data["name"]

    @property
    def world(self) -> str:
        return self._data["world"]

    @property
    def seed_range(self) -> Tuple[int, int]:
        sr = self._data.get("seed_range", [0, 29])
        return (sr[0], sr[1])

    def resolve(self, seed: int) -> TrialConfig:
        """Resolve all randomized parameters for a given seed."""
        rng = np.random.RandomState(seed)
        robot = self._data["robot"]

        peds = []
        for ped_cfg in self._data.get("pedestrians", []):
            # Sample speed
            speed = self._sample(rng, ped_cfg["speed"])

            # Sample start delay
            delay = self._sample(rng, ped_cfg["start_delay"])

            # Sample lateral offset
            lat_offset = self._sample(rng, ped_cfg["lateral_offset"])

            # Adjust waypoints by lateral offset
            raw_wps = ped_cfg["waypoints"]
            adjusted_wps = self._offset_waypoints(raw_wps, lat_offset)

            peds.append(
                PedestrianInstance(
                    id=ped_cfg["id"],
                    speed=speed,
                    start_delay=delay,
                    waypoints=adjusted_wps,
                    spawn_x=adjusted_wps[0][0],
                    spawn_y=adjusted_wps[0][1],
                    spawn_z=0.85,
                )
            )

        return TrialConfig(
            scenario_name=self.name,
            world_file=self.world,
            robot_start_x=robot["start"]["x"],
            robot_start_y=robot["start"]["y"],
            robot_start_yaw=robot["start"]["yaw"],
            goal_x=robot["goal"]["x"],
            goal_y=robot["goal"]["y"],
            timeout=self._data.get("timeout", 60.0),
            seed=seed,
            pedestrians=peds,
        )

    @staticmethod
    def _sample(rng: np.random.RandomState, param: dict) -> float:
        """Sample from a clipped normal distribution."""
        val = rng.normal(param["mean"], param.get("std", 0.0))
        return float(np.clip(val, param.get("min", -np.inf), param.get("max", np.inf)))

    @staticmethod
    def _offset_waypoints(
        waypoints: List[List[float]],
        lateral_offset: float,
    ) -> List[Tuple[float, float]]:
        """Offset waypoints perpendicular to the first segment direction."""
        if len(waypoints) < 2:
            return [(wp[0], wp[1] + lateral_offset) for wp in waypoints]

        # Direction of first segment
        dx = waypoints[1][0] - waypoints[0][0]
        dy = waypoints[1][1] - waypoints[0][1]
        length = np.hypot(dx, dy)
        if length < 1e-6:
            return [(wp[0], wp[1] + lateral_offset) for wp in waypoints]

        # Perpendicular direction (rotate 90 degrees CCW)
        nx = -dy / length
        ny = dx / length

        return [
            (wp[0] + nx * lateral_offset, wp[1] + ny * lateral_offset)
            for wp in waypoints
        ]

    def generate_spawn_commands(
        self, trial: TrialConfig, model_sdf_path: str
    ) -> List[str]:
        """Generate ros2 commands to spawn pedestrian models."""
        cmds = []
        for ped in trial.pedestrians:
            cmd = (
                f"ros2 run ros_gz_sim create "
                f"-world default "
                f"-file {model_sdf_path} "
                f"-name pedestrian_{ped.id} "
                f"-x {ped.spawn_x:.3f} "
                f"-y {ped.spawn_y:.3f} "
                f"-z {ped.spawn_z:.3f}"
            )
            cmds.append(cmd)
        return cmds
