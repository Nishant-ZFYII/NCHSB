#!/usr/bin/env python3
"""
Tests for the ScenarioLoader module.

Test design rationale:
  The scenario loader is the deterministic core of the experiment pipeline.
  If scenario resolution is wrong (e.g., different seeds produce identical
  trials, or parameters exceed their bounds), every downstream trial is
  invalid. These tests catch such bugs before running ~1,710 costly
  simulation trials.

Test categories:
  1. YAML Parsing: Can we load each real scenario file and extract fields?
  2. Seed Reproducibility: Same seed -> identical trial config (bit-exact).
  3. Seed Diversity: Different seeds -> different parameters.
  4. Bound Enforcement: All randomized values stay within [min, max].
  5. Waypoint Offsetting: Lateral offset shifts waypoints perpendicular
     to the walking direction, not along it.
  6. Spawn Commands: Generated spawn commands contain correct coordinates.
  7. Edge Cases: Zero pedestrians, single waypoint, zero-std distributions.
"""

import os
import sys
import math
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from corridor_social_nav_bringup.scenario_loader import (
    ScenarioLoader,
    TrialConfig,
    PedestrianInstance,
)

SCENARIOS_DIR = os.path.join(os.path.dirname(__file__), '..', 'scenarios')


class TestYAMLParsing(unittest.TestCase):
    """Verify all real scenario files load without error."""

    SCENARIO_FILES = [
        'head_on_single.yaml',
        'head_on_group.yaml',
        'doorway_popout.yaml',
        'junction_crossing.yaml',
        'overtaking.yaml',
        'bidirectional.yaml',
    ]

    def test_all_scenarios_load(self):
        for fname in self.SCENARIO_FILES:
            path = os.path.join(SCENARIOS_DIR, fname)
            if not os.path.exists(path):
                self.skipTest(f'{fname} not found')
            loader = ScenarioLoader(path)
            self.assertIsNotNone(loader.name)
            self.assertIsNotNone(loader.world)
            self.assertEqual(len(loader.seed_range), 2)

    def test_head_on_single_fields(self):
        path = os.path.join(SCENARIOS_DIR, 'head_on_single.yaml')
        if not os.path.exists(path):
            self.skipTest('head_on_single.yaml not found')
        loader = ScenarioLoader(path)
        self.assertEqual(loader.name, 'head_on_single')
        self.assertEqual(loader.world, 'corridor_straight.sdf')
        self.assertEqual(loader.seed_range, (0, 29))

    def test_bidirectional_has_four_pedestrians(self):
        path = os.path.join(SCENARIOS_DIR, 'bidirectional.yaml')
        if not os.path.exists(path):
            self.skipTest('bidirectional.yaml not found')
        loader = ScenarioLoader(path)
        trial = loader.resolve(0)
        self.assertEqual(len(trial.pedestrians), 4)


class TestSeedReproducibility(unittest.TestCase):
    """Same seed must produce bit-exact same trial config."""

    def _make_simple_scenario(self):
        data = {
            'scenario': {
                'name': 'test_repro',
                'world': 'test.sdf',
                'robot': {
                    'start': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'goal': {'x': 10.0, 'y': 0.0},
                },
                'pedestrians': [{
                    'id': 1,
                    'waypoints': [[10.0, 0.0], [-10.0, 0.0]],
                    'speed': {'mean': 1.0, 'std': 0.2, 'min': 0.5, 'max': 1.5},
                    'start_delay': {'mean': 1.0, 'std': 0.5, 'min': 0.0, 'max': 3.0},
                    'lateral_offset': {'mean': 0.0, 'std': 0.1, 'min': -0.5, 'max': 0.5},
                }],
                'timeout': 60.0,
                'seed_range': [0, 29],
            }
        }
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        return f.name

    def test_same_seed_same_result(self):
        path = self._make_simple_scenario()
        try:
            loader = ScenarioLoader(path)
            t1 = loader.resolve(42)
            t2 = loader.resolve(42)
            self.assertEqual(t1.pedestrians[0].speed, t2.pedestrians[0].speed)
            self.assertEqual(t1.pedestrians[0].start_delay, t2.pedestrians[0].start_delay)
            self.assertEqual(t1.pedestrians[0].waypoints, t2.pedestrians[0].waypoints)
        finally:
            os.unlink(path)

    def test_different_seeds_different_result(self):
        path = self._make_simple_scenario()
        try:
            loader = ScenarioLoader(path)
            results = set()
            for seed in range(30):
                t = loader.resolve(seed)
                results.add(t.pedestrians[0].speed)
            self.assertGreater(len(results), 1,
                               'All 30 seeds produced identical speed')
        finally:
            os.unlink(path)


class TestBoundEnforcement(unittest.TestCase):
    """All sampled parameters must stay within configured [min, max]."""

    def test_speed_bounds(self):
        data = {
            'scenario': {
                'name': 'test_bounds',
                'world': 'test.sdf',
                'robot': {
                    'start': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'goal': {'x': 10.0, 'y': 0.0},
                },
                'pedestrians': [{
                    'id': 1,
                    'waypoints': [[5.0, 0.0], [-5.0, 0.0]],
                    'speed': {'mean': 1.0, 'std': 0.5, 'min': 0.3, 'max': 1.5},
                    'start_delay': {'mean': 1.0, 'std': 2.0, 'min': 0.0, 'max': 5.0},
                    'lateral_offset': {'mean': 0.0, 'std': 0.5, 'min': -0.3, 'max': 0.3},
                }],
                'timeout': 60.0,
            }
        }
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        try:
            loader = ScenarioLoader(f.name)
            for seed in range(100):
                trial = loader.resolve(seed)
                ped = trial.pedestrians[0]
                self.assertGreaterEqual(ped.speed, 0.3,
                                        f'seed={seed}: speed={ped.speed} < 0.3')
                self.assertLessEqual(ped.speed, 1.5,
                                     f'seed={seed}: speed={ped.speed} > 1.5')
                self.assertGreaterEqual(ped.start_delay, 0.0,
                                        f'seed={seed}: delay={ped.start_delay} < 0')
                self.assertLessEqual(ped.start_delay, 5.0,
                                     f'seed={seed}: delay={ped.start_delay} > 5')
        finally:
            os.unlink(f.name)


class TestWaypointOffsetting(unittest.TestCase):
    """Lateral offset shifts waypoints perpendicular to walking direction."""

    def test_horizontal_walk_offset_shifts_y(self):
        """Walking along X-axis: offset should shift Y only."""
        waypoints = [[0.0, 0.0], [10.0, 0.0]]
        offset = 0.5
        result = ScenarioLoader._offset_waypoints(waypoints, offset)
        for i, (rx, ry) in enumerate(result):
            self.assertAlmostEqual(rx, waypoints[i][0], places=6,
                                   msg=f'X should not change for horizontal walk')
            self.assertAlmostEqual(ry, waypoints[i][1] + offset, places=6,
                                   msg=f'Y should shift by offset={offset}')

    def test_vertical_walk_offset_shifts_x(self):
        """Walking along Y-axis: offset should shift X only."""
        waypoints = [[0.0, 0.0], [0.0, 10.0]]
        offset = 0.3
        result = ScenarioLoader._offset_waypoints(waypoints, offset)
        for i, (rx, ry) in enumerate(result):
            self.assertAlmostEqual(rx, waypoints[i][0] - offset, places=6,
                                   msg='X should shift by -offset for upward walk')
            self.assertAlmostEqual(ry, waypoints[i][1], places=6,
                                   msg='Y should not change for vertical walk')

    def test_zero_offset_no_change(self):
        waypoints = [[1.0, 2.0], [5.0, 7.0]]
        result = ScenarioLoader._offset_waypoints(waypoints, 0.0)
        for i, (rx, ry) in enumerate(result):
            self.assertAlmostEqual(rx, waypoints[i][0], places=10)
            self.assertAlmostEqual(ry, waypoints[i][1], places=10)

    def test_negative_offset(self):
        """Negative offset should shift the other direction."""
        waypoints = [[0.0, 0.0], [10.0, 0.0]]
        pos_result = ScenarioLoader._offset_waypoints(waypoints, 0.5)
        neg_result = ScenarioLoader._offset_waypoints(waypoints, -0.5)
        self.assertAlmostEqual(pos_result[0][1], 0.5, places=6)
        self.assertAlmostEqual(neg_result[0][1], -0.5, places=6)

    def test_single_waypoint_defaults_to_y_shift(self):
        waypoints = [[3.0, 4.0]]
        result = ScenarioLoader._offset_waypoints(waypoints, 0.2)
        self.assertAlmostEqual(result[0][0], 3.0, places=6)
        self.assertAlmostEqual(result[0][1], 4.2, places=6)


class TestSpawnCommands(unittest.TestCase):
    """Generated spawn commands should contain correct model info."""

    def test_spawn_command_format(self):
        trial = TrialConfig(
            scenario_name='test',
            world_file='test.sdf',
            robot_start_x=0.0, robot_start_y=0.0, robot_start_yaw=0.0,
            goal_x=10.0, goal_y=0.0,
            timeout=60.0, seed=0,
            pedestrians=[
                PedestrianInstance(
                    id=1, speed=1.0, start_delay=0.0,
                    waypoints=[(5.0, 0.3)], spawn_x=5.0, spawn_y=0.3, spawn_z=0.85),
                PedestrianInstance(
                    id=2, speed=0.8, start_delay=1.0,
                    waypoints=[(3.0, -0.2)], spawn_x=3.0, spawn_y=-0.2, spawn_z=0.85),
            ],
        )
        data = {'scenario': {
            'name': 'test', 'world': 'test.sdf',
            'robot': {'start': {'x': 0, 'y': 0, 'yaw': 0}, 'goal': {'x': 10, 'y': 0}},
            'pedestrians': [], 'timeout': 60.0,
        }}
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        try:
            loader = ScenarioLoader(f.name)
            cmds = loader.generate_spawn_commands(trial, '/path/to/model.sdf')
            self.assertEqual(len(cmds), 2)
            self.assertIn('pedestrian_1', cmds[0])
            self.assertIn('pedestrian_2', cmds[1])
            self.assertIn('/path/to/model.sdf', cmds[0])
            self.assertIn('-x 5.000', cmds[0])
            self.assertIn('-y 0.300', cmds[0])
            self.assertIn('-z 0.850', cmds[0])
        finally:
            os.unlink(f.name)


class TestTrialConfig(unittest.TestCase):
    """Test TrialConfig fields are set correctly."""

    def test_robot_config_from_yaml(self):
        data = {
            'scenario': {
                'name': 'test_robot',
                'world': 'corridor_straight.sdf',
                'robot': {
                    'start': {'x': -9.0, 'y': 1.0, 'yaw': 0.5},
                    'goal': {'x': 9.0, 'y': -1.0},
                },
                'pedestrians': [],
                'timeout': 45.0,
                'seed_range': [5, 10],
            }
        }
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        try:
            loader = ScenarioLoader(f.name)
            self.assertEqual(loader.seed_range, (5, 10))
            trial = loader.resolve(7)
            self.assertEqual(trial.scenario_name, 'test_robot')
            self.assertEqual(trial.world_file, 'corridor_straight.sdf')
            self.assertAlmostEqual(trial.robot_start_x, -9.0)
            self.assertAlmostEqual(trial.robot_start_y, 1.0)
            self.assertAlmostEqual(trial.robot_start_yaw, 0.5)
            self.assertAlmostEqual(trial.goal_x, 9.0)
            self.assertAlmostEqual(trial.goal_y, -1.0)
            self.assertAlmostEqual(trial.timeout, 45.0)
            self.assertEqual(trial.seed, 7)
            self.assertEqual(len(trial.pedestrians), 0)
        finally:
            os.unlink(f.name)


class TestEdgeCases(unittest.TestCase):
    """Edge cases that could break the pipeline."""

    def test_zero_std_gives_mean(self):
        """Zero std should always return the mean value."""
        data = {
            'scenario': {
                'name': 'test_zero_std',
                'world': 'test.sdf',
                'robot': {
                    'start': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'goal': {'x': 10.0, 'y': 0.0},
                },
                'pedestrians': [{
                    'id': 1,
                    'waypoints': [[5.0, 0.0], [-5.0, 0.0]],
                    'speed': {'mean': 1.0, 'std': 0.0, 'min': 0.5, 'max': 1.5},
                    'start_delay': {'mean': 2.0, 'std': 0.0, 'min': 0.0, 'max': 5.0},
                    'lateral_offset': {'mean': 0.1, 'std': 0.0, 'min': -0.5, 'max': 0.5},
                }],
                'timeout': 60.0,
            }
        }
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        try:
            loader = ScenarioLoader(f.name)
            for seed in range(10):
                trial = loader.resolve(seed)
                self.assertAlmostEqual(trial.pedestrians[0].speed, 1.0, places=10)
                self.assertAlmostEqual(trial.pedestrians[0].start_delay, 2.0, places=10)
        finally:
            os.unlink(f.name)

    def test_spawn_z_is_always_0_85(self):
        data = {
            'scenario': {
                'name': 'test_z',
                'world': 'test.sdf',
                'robot': {
                    'start': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'goal': {'x': 10.0, 'y': 0.0},
                },
                'pedestrians': [{
                    'id': 1,
                    'waypoints': [[5.0, 0.0], [-5.0, 0.0]],
                    'speed': {'mean': 1.0, 'std': 0.0, 'min': 0.5, 'max': 1.5},
                    'start_delay': {'mean': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0},
                    'lateral_offset': {'mean': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0},
                }],
                'timeout': 60.0,
            }
        }
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        try:
            loader = ScenarioLoader(f.name)
            trial = loader.resolve(0)
            self.assertAlmostEqual(trial.pedestrians[0].spawn_z, 0.85)
        finally:
            os.unlink(f.name)

    def test_collinear_waypoints(self):
        """All waypoints on a line should still offset correctly."""
        waypoints = [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]]
        result = ScenarioLoader._offset_waypoints(waypoints, 0.5)
        for rx, ry in result:
            self.assertAlmostEqual(ry, 0.5, places=6)


class TestMultiPedSeedIndependence(unittest.TestCase):
    """Each pedestrian in a multi-ped scenario should get
    different randomized parameters within the same seed."""

    def test_group_scenario_diversity(self):
        path = os.path.join(SCENARIOS_DIR, 'head_on_group.yaml')
        if not os.path.exists(path):
            self.skipTest('head_on_group.yaml not found')
        loader = ScenarioLoader(path)
        trial = loader.resolve(0)
        speeds = [p.speed for p in trial.pedestrians]
        delays = [p.start_delay for p in trial.pedestrians]
        self.assertEqual(len(trial.pedestrians), 3)
        self.assertTrue(
            len(set(speeds)) > 1 or len(set(delays)) > 1,
            'Group pedestrians should have some parameter diversity'
        )


if __name__ == '__main__':
    unittest.main()
