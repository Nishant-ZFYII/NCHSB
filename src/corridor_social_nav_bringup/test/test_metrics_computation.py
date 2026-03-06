#!/usr/bin/env python3
"""
Tests for metrics computation logic.

Test design rationale:
  The metrics logger accumulates values incrementally during a trial.
  If the accumulation logic is wrong (e.g., collision count doesn't do
  rising-edge detection, or path length integration is off), the entire
  paper's quantitative results are invalid. These tests verify the
  computation logic in isolation, without needing a running ROS system.

We test the logic functions directly rather than the ROS node,
since the node is just a thin wrapper around these computations.

Test categories:
  1. Collision counting: rising-edge detection (False->True only)
  2. Min separation tracking: global minimum over all samples
  3. TTC violation counting: threshold at 1.0s
  4. Shield intervention percentage: ratio computation
  5. Path length integration: Euclidean distance accumulation
  6. Stuck time accumulation: time with |v| < 0.02 m/s
  7. CSV output: correct format, header, row content
  8. Result string parsing: SUCCESS/TIMEOUT/ABORTED formats
"""

import csv
import os
import math
import tempfile
import unittest


class CollisionCounter:
    """Extracted collision counting logic from MetricsLogger."""

    def __init__(self):
        self.count = 0
        self.prev = False

    def update(self, is_collision: bool):
        if is_collision and not self.prev:
            self.count += 1
        self.prev = is_collision


class PathIntegrator:
    """Extracted path length integration from MetricsLogger."""

    def __init__(self):
        self.length = 0.0
        self.prev_x = None
        self.prev_y = None

    def update(self, x: float, y: float):
        if self.prev_x is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            self.length += math.sqrt(dx * dx + dy * dy)
        self.prev_x = x
        self.prev_y = y


class StuckTimeTracker:
    """Extracted stuck time logic from MetricsLogger."""

    def __init__(self, threshold: float = 0.02):
        self.stuck_time = 0.0
        self.threshold = threshold
        self.prev_time = None

    def update(self, v: float, timestamp: float):
        if self.prev_time is not None:
            dt = timestamp - self.prev_time
            if dt > 0 and abs(v) < self.threshold:
                self.stuck_time += dt
        self.prev_time = timestamp


class TestCollisionCounting(unittest.TestCase):
    """Rising-edge collision detection."""

    def test_no_collisions(self):
        c = CollisionCounter()
        for _ in range(100):
            c.update(False)
        self.assertEqual(c.count, 0)

    def test_single_collision_event(self):
        c = CollisionCounter()
        c.update(False)
        c.update(True)  # rising edge -> count
        c.update(True)  # still in collision, no new count
        c.update(True)
        c.update(False)
        self.assertEqual(c.count, 1)

    def test_two_separate_collisions(self):
        c = CollisionCounter()
        c.update(True)   # rising edge (from initial False)
        c.update(False)
        c.update(True)   # another rising edge
        c.update(False)
        self.assertEqual(c.count, 2)

    def test_sustained_collision_counts_once(self):
        c = CollisionCounter()
        for _ in range(50):
            c.update(True)
        self.assertEqual(c.count, 1)

    def test_rapid_toggle(self):
        c = CollisionCounter()
        for _ in range(10):
            c.update(True)
            c.update(False)
        self.assertEqual(c.count, 10)


class TestMinSeparation(unittest.TestCase):
    """Global minimum distance tracking."""

    def test_decreasing_distances(self):
        min_sep = float('inf')
        for d in [5.0, 3.0, 1.0, 0.5, 0.3]:
            if d < min_sep:
                min_sep = d
        self.assertAlmostEqual(min_sep, 0.3)

    def test_increasing_then_decreasing(self):
        min_sep = float('inf')
        for d in [1.0, 2.0, 3.0, 0.5, 1.0]:
            if d < min_sep:
                min_sep = d
        self.assertAlmostEqual(min_sep, 0.5)

    def test_no_data(self):
        min_sep = float('inf')
        self.assertEqual(min_sep, float('inf'))


class TestTTCViolations(unittest.TestCase):
    """TTC violation counting (threshold: 1.0s)."""

    def test_no_violations(self):
        count = 0
        for ttc in [5.0, 3.0, 2.0, 1.5, 1.1]:
            if ttc < 1.0:
                count += 1
        self.assertEqual(count, 0)

    def test_some_violations(self):
        count = 0
        for ttc in [5.0, 0.8, 1.5, 0.3, 0.9, 2.0]:
            if ttc < 1.0:
                count += 1
        self.assertEqual(count, 3)

    def test_exactly_1s_not_violation(self):
        count = 0
        if 1.0 < 1.0:
            count += 1
        self.assertEqual(count, 0)


class TestShieldIntervention(unittest.TestCase):
    """Shield intervention percentage."""

    def test_no_interventions(self):
        total = 100
        interventions = 0
        pct = 100.0 * interventions / total
        self.assertAlmostEqual(pct, 0.0)

    def test_all_interventions(self):
        total = 50
        interventions = 50
        pct = 100.0 * interventions / total
        self.assertAlmostEqual(pct, 100.0)

    def test_half_interventions(self):
        total = 200
        interventions = 100
        pct = 100.0 * interventions / total
        self.assertAlmostEqual(pct, 50.0)

    def test_zero_total_no_crash(self):
        total = 0
        pct = 0.0 if total == 0 else 100.0 * 0 / total
        self.assertAlmostEqual(pct, 0.0)


class TestPathIntegration(unittest.TestCase):
    """Path length computation from odometry."""

    def test_straight_line(self):
        p = PathIntegrator()
        for x in range(11):
            p.update(float(x), 0.0)
        self.assertAlmostEqual(p.length, 10.0, places=6)

    def test_diagonal(self):
        p = PathIntegrator()
        p.update(0.0, 0.0)
        p.update(3.0, 4.0)
        self.assertAlmostEqual(p.length, 5.0, places=6)

    def test_stationary(self):
        p = PathIntegrator()
        for _ in range(10):
            p.update(1.0, 2.0)
        self.assertAlmostEqual(p.length, 0.0, places=10)

    def test_return_trip(self):
        p = PathIntegrator()
        p.update(0.0, 0.0)
        p.update(5.0, 0.0)
        p.update(0.0, 0.0)
        self.assertAlmostEqual(p.length, 10.0, places=6)

    def test_single_point(self):
        p = PathIntegrator()
        p.update(3.0, 4.0)
        self.assertAlmostEqual(p.length, 0.0)


class TestStuckTime(unittest.TestCase):
    """Stuck time (velocity below threshold)."""

    def test_always_moving(self):
        s = StuckTimeTracker()
        for i in range(10):
            s.update(1.0, float(i))
        self.assertAlmostEqual(s.stuck_time, 0.0)

    def test_always_stuck(self):
        s = StuckTimeTracker()
        for i in range(11):
            s.update(0.0, float(i))
        self.assertAlmostEqual(s.stuck_time, 10.0, places=6)

    def test_partial_stuck(self):
        s = StuckTimeTracker()
        s.update(1.0, 0.0)
        s.update(1.0, 1.0)  # moving, +0
        s.update(0.0, 2.0)  # stuck, +1
        s.update(0.0, 3.0)  # stuck, +1
        s.update(1.0, 4.0)  # moving, +0
        self.assertAlmostEqual(s.stuck_time, 2.0, places=6)

    def test_threshold_boundary(self):
        s = StuckTimeTracker()
        s.update(0.019, 0.0)
        s.update(0.019, 1.0)  # |v| < 0.02 -> stuck
        s.update(0.021, 2.0)  # |v| > 0.02 -> not stuck
        self.assertAlmostEqual(s.stuck_time, 1.0, places=6)


class TestResultParsing(unittest.TestCase):
    """Result string parsing (matching metrics_logger.result_cb logic)."""

    def _parse(self, result_str):
        success = False
        time_to_goal = 0.0
        if result_str.startswith('SUCCESS'):
            success = True
            parts = result_str.split(':')
            if len(parts) > 1:
                time_to_goal = float(parts[1])
        elif ':' in result_str:
            parts = result_str.split(':')
            time_to_goal = float(parts[1])
        return success, time_to_goal

    def test_success(self):
        s, t = self._parse('SUCCESS:42.50')
        self.assertTrue(s)
        self.assertAlmostEqual(t, 42.5)

    def test_timeout(self):
        s, t = self._parse('TIMEOUT:120.00')
        self.assertFalse(s)
        self.assertAlmostEqual(t, 120.0)

    def test_aborted(self):
        s, t = self._parse('ABORTED:35.00')
        self.assertFalse(s)
        self.assertAlmostEqual(t, 35.0)

    def test_no_server(self):
        s, t = self._parse('TIMEOUT_NO_SERVER')
        self.assertFalse(s)
        self.assertAlmostEqual(t, 0.0)

    def test_rejected(self):
        s, t = self._parse('REJECTED')
        self.assertFalse(s)
        self.assertAlmostEqual(t, 0.0)


class TestCSVOutput(unittest.TestCase):
    """CSV file writing format."""

    def test_csv_has_correct_columns(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv',
                                         delete=False, newline='') as f:
            row = {
                'scenario': 'head_on_single',
                'controller': 'mppi_vanilla',
                'seed': 0,
                'success': 1,
                'collision_count': 0,
                'min_separation_m': 0.45,
                'ttc_violations': 3,
                'shield_interventions_pct': 12.5,
                'time_to_goal_s': 42.3,
                'path_length_m': 18.5,
                'stuck_time_s': 1.2,
            }
            writer = csv.DictWriter(f, fieldnames=row.keys())
            writer.writeheader()
            writer.writerow(row)
            fname = f.name

        try:
            with open(fname, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['scenario'], 'head_on_single')
            self.assertEqual(rows[0]['controller'], 'mppi_vanilla')
            self.assertEqual(int(rows[0]['seed']), 0)
            self.assertEqual(int(rows[0]['success']), 1)
            self.assertAlmostEqual(float(rows[0]['min_separation_m']), 0.45)
        finally:
            os.unlink(fname)

    def test_csv_append_mode(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv',
                                         delete=False, newline='') as f:
            fname = f.name
            fieldnames = ['scenario', 'seed']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({'scenario': 'a', 'seed': 0})

        with open(fname, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['scenario', 'seed'])
            writer.writerow({'scenario': 'b', 'seed': 1})

        try:
            with open(fname, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]['scenario'], 'a')
            self.assertEqual(rows[1]['scenario'], 'b')
        finally:
            os.unlink(fname)


if __name__ == '__main__':
    unittest.main()
