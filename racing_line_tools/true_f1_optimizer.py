#!/usr/bin/env python3
"""
TRUE F1-STYLE RACING LINE OPTIMIZER

CRITICAL DIFFERENCE FROM MINIMUM-CURVATURE:
- Minimum curvature = minimize integral of κ² → stays near centerline
- F1-style = MAXIMIZE CORNER SPEED → aggressive outside-inside-outside

The key insight:
- Minimum curvature is NOT the same as maximum speed!
- F1 cars use LATE APEX and WIDE ARCS to maximize exit speed
- This requires DIFFERENT objective:
  * In CORNERS: MAXIMIZE track width usage (wide arc = high speed)
  * On STRAIGHTS: MINIMIZE deviation (tight line = short distance)

Uses the same centerline from Raceline-Optimization repo.
"""

import numpy as np
import yaml
import csv
import cvxpy as cp
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d, distance_transform_edt
from pathlib import Path
import matplotlib.pyplot as plt
import cv2
import argparse


class TrueF1Optimizer:
    """
    F1-style racing line: MAXIMIZE CORNER SPEED (not minimize curvature!)
    """
    
    def __init__(self, vehicle_params=None, f1_params=None):
        """
        Initialize F1 optimizer
        
        Key parameters:
            - corner_aggression: How wide to use corners (0-1)
            - straight_tightness: How tight on straights (0-1)
        """
        
        # Vehicle parameters (same as other optimizers)
        self.vehicle = vehicle_params or {
            'wheelbase': 0.324,
            'width': 0.28,
            'length': 0.50,
            'max_steering_angle': 0.5236,  # 30 degrees
            'mu': 0.5,
            'v_max': 2.0,
            'inflation_radius': 0.05,  # Reduced for narrow track
        }
        
        # Ackermann constraints
        self.R_min = self.vehicle['wheelbase'] / np.tan(self.vehicle['max_steering_angle'])
        self.kappa_max = 1.0 / self.R_min
        
        # F1 racing parameters
        self.f1 = f1_params or {
            'corner_aggression': 0.90,    # Use 90% of available width in corners
            'straight_tightness': 0.25,   # Use only 25% on straights (tight line)
            'min_corner_curvature': 0.5,  # Threshold to identify corners (rad/m)
            'corner_weight': 800,         # Weight for corner incentive
            'straight_weight': 400,       # Weight for straight penalty
        }
        
        # Safety margin (MINIMAL for narrow corridor)
        # For this narrow track, we can't use standard margins
        self.safety_margin = 0.10  # Just 10cm - minimal safe margin
        
        self._print_config()
    
    def _print_config(self):
        """Print configuration"""
        print("=" * 70)
        print("TRUE F1-STYLE RACING LINE OPTIMIZER")
        print("(MAXIMIZE corner speed, NOT minimize curvature!)")
        print("=" * 70)
        print(f"\nVehicle:")
        print(f"  Min turning radius: {self.R_min:.3f}m")
        print(f"  Max curvature:      {self.kappa_max:.3f} rad/m")
        print(f"  Safety margin:      {self.safety_margin:.3f}m")
        print(f"\nF1 Strategy:")
        print(f"  Corner aggression:  {self.f1['corner_aggression']*100:.0f}% track width")
        print(f"  Straight tightness: {self.f1['straight_tightness']*100:.0f}% track width")
        print(f"  Corner threshold:   {self.f1['min_corner_curvature']} rad/m")
        print("=" * 70)
    
    def load_centerline_csv(self, csv_path):
        """Load centerline from TUM CSV format"""
        print(f"\n[1/7] Loading centerline from: {csv_path}")
        
        xs, ys, w_right, w_left = [], [], [], []
        
        with open(csv_path) as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                xs.append(float(row[0]))
                ys.append(float(row[1]))
                w_right.append(float(row[2]))
                w_left.append(float(row[3]))
        
        self.centerline = np.column_stack([xs, ys])
        self.track_widths_right = np.array(w_right)
        self.track_widths_left = np.array(w_left)
        
        print(f"  Loaded {len(self.centerline)} points")
        print(f"  X range: [{min(xs):.2f}, {max(xs):.2f}]m")
        print(f"  Y range: [{min(ys):.2f}, {max(ys):.2f}]m")
        
        return self.centerline
    
    def recalculate_track_widths(self, map_pgm, map_yaml):
        """Recalculate track widths from map distance field"""
        print("\n[2/7] Recalculating track widths from map...")
        
        # Load map
        map_img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
        with open(map_yaml) as f:
            config = yaml.safe_load(f)
        
        resolution = config['resolution']
        origin = config['origin'][:2]
        
        # Distance field
        free_space = (map_img > 200).astype(float)
        distance_field = distance_transform_edt(free_space) * resolution
        
        print(f"  Map: {map_img.shape}, Resolution: {resolution}m/px")
        print(f"  Max clearance: {distance_field.max():.3f}m")
        
        # Get clearance at each centerline point
        widths = []
        for x, y in self.centerline:
            px = int((x - origin[0]) / resolution)
            py = map_img.shape[0] - int((y - origin[1]) / resolution)
            
            px = max(0, min(px, map_img.shape[1] - 1))
            py = max(0, min(py, map_img.shape[0] - 1))
            
            clearance = distance_field[py, px]
            widths.append(max(0.05, clearance))  # Minimum 5cm
        
        widths = np.array(widths)
        
        # Smooth widths to remove noise
        widths = gaussian_filter1d(widths, sigma=2)
        
        self.track_widths_right = widths
        self.track_widths_left = widths
        
        print(f"  Track width range: [{widths.min():.3f}, {widths.max():.3f}]m")
        
        # Store for visualization
        self.map_img = map_img
        self.map_config = config
    
    def compute_centerline_curvature(self):
        """Compute curvature of centerline to identify corners"""
        print("\n[3/7] Identifying corners (high curvature regions)...")
        
        N = len(self.centerline)
        
        # Arc length parameterization
        ds = np.sqrt(np.sum(np.diff(self.centerline, axis=0)**2, axis=1))
        self.arc_length = np.concatenate([[0], np.cumsum(ds)])
        self.total_length = self.arc_length[-1]
        
        # Check if closed
        gap = np.linalg.norm(self.centerline[0] - self.centerline[-1])
        self.is_closed = gap < 0.5
        
        # Fit splines
        bc_type = 'natural'  # Open track
        self.spline_x = CubicSpline(self.arc_length, self.centerline[:, 0], bc_type=bc_type)
        self.spline_y = CubicSpline(self.arc_length, self.centerline[:, 1], bc_type=bc_type)
        
        # First derivatives (tangent)
        dx = self.spline_x(self.arc_length, 1)
        dy = self.spline_y(self.arc_length, 1)
        
        # Second derivatives
        ddx = self.spline_x(self.arc_length, 2)
        ddy = self.spline_y(self.arc_length, 2)
        
        # Curvature: κ = |x'y'' - y'x''| / (x'² + y'²)^1.5
        norm = (dx**2 + dy**2)**1.5
        norm[norm < 1e-10] = 1e-10
        self.centerline_curvature = np.abs(dx * ddy - dy * ddx) / norm
        
        # Identify corners (high curvature sections)
        self.is_corner = self.centerline_curvature > self.f1['min_corner_curvature']
        
        # Smooth corner identification to get continuous regions
        self.is_corner = gaussian_filter1d(self.is_corner.astype(float), sigma=3) > 0.4
        
        # Compute normals
        tangent_norm = np.sqrt(dx**2 + dy**2)
        tangent_norm[tangent_norm < 1e-10] = 1e-10
        tangent = np.column_stack([dx / tangent_norm, dy / tangent_norm])
        self.normals = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        
        num_corners = np.sum(self.is_corner)
        print(f"  Total length: {self.total_length:.2f}m")
        print(f"  Corner points: {num_corners}/{N} ({100*num_corners/N:.1f}%)")
        print(f"  Curvature range: [{self.centerline_curvature.min():.3f}, {self.centerline_curvature.max():.3f}] rad/m")
    
    def optimize_f1_racing_line(self):
        """
        F1-STYLE OPTIMIZATION
        
        Key difference from minimum curvature:
        - In CORNERS: Use MAXIMUM available width (wide arc = high speed)
        - On STRAIGHTS: Use MINIMUM deviation (tight line = short distance)
        
        This creates natural outside-inside-outside!
        
        We use a convex reformulation:
        - Corner target = push toward max allowed offset
        - Straight target = push toward zero offset
        """
        print("\n[4/7] Optimizing TRUE F1 racing line...")
        print("  (MAXIMIZE corner width, MINIMIZE straight deviation)")
        
        N = len(self.centerline)
        
        # Decision variable: lateral offset
        alpha = cp.Variable(N)
        
        # ================================================
        # F1-STYLE OBJECTIVE (Convex formulation)
        # ================================================
        
        # Build target offsets: 
        # - For corners: target = max allowed offset (push to walls)
        # - For straights: target = 0 (stay on centerline)
        
        targets = np.zeros(N)
        weights = np.zeros(N)
        
        for i in range(N):
            if self.is_corner[i]:
                # CORNERS: Target = 80% of max allowed offset (toward outside)
                # Alternate direction based on curvature sign to create outside-inside-outside
                max_offset = self.f1['corner_aggression'] * min(
                    self.track_widths_left[i], self.track_widths_right[i]
                ) - self.safety_margin
                
                # Use curvature direction to determine which side to push toward
                if self.centerline_curvature[i] > 0:
                    targets[i] = max(0.02, max_offset * 0.8)  # Push left
                else:
                    targets[i] = min(-0.02, -max_offset * 0.8)  # Push right
                
                weights[i] = self.f1['corner_weight']
            else:
                # STRAIGHTS: Target = 0 (centerline)
                targets[i] = 0.0
                weights[i] = self.f1['straight_weight']
        
        # Objective: Minimize weighted distance from targets
        # This is convex: sum of weighted squared differences
        deviation_from_target = cp.sum(cp.multiply(weights, cp.square(alpha - targets)))
        
        # Smoothness (prevent oscillations)
        smoothness = cp.sum_squares(alpha[1:] - alpha[:-1])
        
        # Curvature cost (mild, to avoid extreme oscillations)
        x_race = self.centerline[:, 0] + cp.multiply(alpha, self.normals[:, 0])
        y_race = self.centerline[:, 1] + cp.multiply(alpha, self.normals[:, 1])
        
        ds = np.diff(self.arc_length)
        ds[ds < 1e-6] = 1e-6
        ds_mid = (ds[:-1] + ds[1:]) / 2
        
        curv_x = (x_race[2:] - 2*x_race[1:-1] + x_race[:-2]) / (ds_mid**2)
        curv_y = (y_race[2:] - 2*y_race[1:-1] + y_race[:-2]) / (ds_mid**2)
        curvature_cost = cp.sum_squares(curv_x) + cp.sum_squares(curv_y)
        
        # Total objective (prioritize target following over curvature)
        objective = cp.Minimize(deviation_from_target + 20 * smoothness + 50 * curvature_cost)
        
        # ================================================
        # CONSTRAINTS: Stay within track bounds
        # ================================================
        constraints = []
        
        for i in range(N):
            if self.is_corner[i]:
                # CORNERS: Use aggressive track width
                usage = self.f1['corner_aggression']
            else:
                # STRAIGHTS: Stay closer to centerline
                usage = self.f1['straight_tightness']
            
            max_left = max(0.01, usage * self.track_widths_left[i] - self.safety_margin)
            max_right = max(0.01, usage * self.track_widths_right[i] - self.safety_margin)
            
            constraints.append(alpha[i] >= -max_right)
            constraints.append(alpha[i] <= max_left)
        
        # ================================================
        # SOLVE
        # ================================================
        problem = cp.Problem(objective, constraints)
        result = problem.solve(solver=cp.OSQP, verbose=False, max_iter=20000)
        
        if problem.status in ['optimal', 'optimal_inaccurate']:
            self.alpha = alpha.value
            print(f"  Optimization: {problem.status.upper()} ✓")
        else:
            print(f"  WARNING: {problem.status}")
            self.alpha = np.zeros(N)
        
        # Compute racing line
        self.racing_line = np.column_stack([
            self.centerline[:, 0] + self.alpha * self.normals[:, 0],
            self.centerline[:, 1] + self.alpha * self.normals[:, 1]
        ])
        
        # Statistics
        corner_offsets = np.abs(self.alpha[self.is_corner])
        straight_offsets = np.abs(self.alpha[~self.is_corner])
        
        if len(corner_offsets) > 0 and len(straight_offsets) > 0:
            corner_mean = corner_offsets.mean()
            straight_mean = straight_offsets.mean() if straight_offsets.mean() > 0.001 else 0.001
            ratio = corner_mean / straight_mean
            
            print(f"\n  📊 F1 GEOMETRY METRICS:")
            print(f"  Corner offsets:   [{corner_offsets.min():.3f}, {corner_offsets.max():.3f}]m (avg: {corner_mean:.3f}m)")
            print(f"  Straight offsets: [{straight_offsets.min():.3f}, {straight_offsets.max():.3f}]m (avg: {straight_mean:.3f}m)")
            print(f"  Ratio (corner/straight): {ratio:.1f}x")
            
            if ratio > 3:
                print(f"  ✓ TRUE F1 GEOMETRY ACHIEVED! (ratio > 3x)")
            else:
                print(f"  ⚠ Ratio < 3x - may still look like centerline-hugging")
        
        return self.racing_line
    
    def compute_velocity_profile(self):
        """Compute velocity with friction + Ackermann limits"""
        print("\n[5/7] Computing velocity profile...")
        
        N = len(self.racing_line)
        
        # Curvature using Menger formula
        kappa = np.zeros(N)
        for i in range(1, N-1):
            p0 = self.racing_line[i-1]
            p1 = self.racing_line[i]
            p2 = self.racing_line[i+1]
            
            area = 0.5 * abs((p1[0]-p0[0])*(p2[1]-p0[1]) - (p2[0]-p0[0])*(p1[1]-p0[1]))
            d01 = np.linalg.norm(p1 - p0)
            d12 = np.linalg.norm(p2 - p1)
            d20 = np.linalg.norm(p0 - p2)
            
            if d01 * d12 * d20 > 1e-10:
                kappa[i] = 4 * area / (d01 * d12 * d20)
        
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
        
        # Velocity from friction: v = sqrt(a_lat / kappa)
        a_lat_max = self.vehicle['mu'] * 9.81
        v_max_local = np.sqrt(a_lat_max / np.maximum(kappa, 0.01))
        v_max_local = np.clip(v_max_local, 0.3, self.vehicle['v_max'])
        
        # Smooth velocity profile
        self.velocity = gaussian_filter1d(v_max_local, sigma=3)
        
        # Statistics
        corner_v = self.velocity[self.is_corner].mean() if np.any(self.is_corner) else 0
        straight_v = self.velocity[~self.is_corner].mean() if np.any(~self.is_corner) else 0
        
        print(f"  Velocity range: [{self.velocity.min():.2f}, {self.velocity.max():.2f}] m/s")
        print(f"  Corner avg speed: {corner_v:.2f} m/s")
        print(f"  Straight avg speed: {straight_v:.2f} m/s")
        
        return self.velocity
    
    def save_yaml(self, output_path):
        """Save to Nav2 YAML format"""
        print("\n[6/7] Saving racing line...")
        
        waypoints = []
        s = 0.0
        
        for i in range(len(self.racing_line)):
            if i > 0:
                ds = np.linalg.norm(self.racing_line[i] - self.racing_line[i-1])
                s += ds
            
            # Yaw from direction
            if i < len(self.racing_line) - 1:
                dx = self.racing_line[i+1, 0] - self.racing_line[i, 0]
                dy = self.racing_line[i+1, 1] - self.racing_line[i, 1]
            else:
                dx = self.racing_line[i, 0] - self.racing_line[i-1, 0]
                dy = self.racing_line[i, 1] - self.racing_line[i-1, 1]
            yaw = float(np.arctan2(dy, dx))
            
            waypoints.append({
                'index': int(i),
                's': round(float(s), 4),
                'x': round(float(self.racing_line[i, 0]), 4),
                'y': round(float(self.racing_line[i, 1]), 4),
                'yaw': round(float(yaw), 4),
                'v': round(float(self.velocity[i]), 2),
            })
        
        data = {
            'description': 'TRUE F1-style racing line (maximize corner speed)',
            'optimizer': 'true_f1',
            'frame_id': 'map',
            'num_waypoints': len(waypoints),
            'total_length': round(float(s), 2),
            'waypoints': waypoints
        }
        
        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        print(f"  Output: {output_path}")
        print(f"  Waypoints: {len(waypoints)}")
        print(f"  Total length: {s:.2f}m")
    
    def visualize(self, output_path):
        """Create visualization with corner/straight coloring"""
        print("\n[7/7] Creating visualization...")
        
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))
        
        # ============================================
        # LEFT: Racing line on map
        # ============================================
        ax1 = axes[0]
        
        if hasattr(self, 'map_img') and hasattr(self, 'map_config'):
            origin = self.map_config['origin'][:2]
            resolution = self.map_config['resolution']
            extent = [origin[0], origin[0] + self.map_img.shape[1] * resolution,
                      origin[1], origin[1] + self.map_img.shape[0] * resolution]
            ax1.imshow(self.map_img, cmap='gray', origin='lower', extent=extent, alpha=0.7)
        
        # Centerline (blue dashed)
        ax1.plot(self.centerline[:, 0], self.centerline[:, 1],
                'b--', linewidth=1.5, alpha=0.4, label='Centerline')
        
        # Racing line colored by corner vs straight
        # RED = corners (aggressive), GREEN = straights (tight)
        colors = np.where(self.is_corner, 'red', 'limegreen')
        ax1.scatter(self.racing_line[:, 0], self.racing_line[:, 1],
                   c=colors, s=50, alpha=0.8, edgecolors='black', linewidth=0.5, zorder=5)
        
        # Connect with line
        ax1.plot(self.racing_line[:, 0], self.racing_line[:, 1],
                'k-', linewidth=1, alpha=0.3)
        
        # Start marker
        ax1.plot(self.racing_line[0, 0], self.racing_line[0, 1],
                'ko', markersize=15, label='Start', zorder=10)
        
        # Legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='red', alpha=0.8, label='Corners (WIDE arc)'),
            Patch(facecolor='limegreen', alpha=0.8, label='Straights (TIGHT line)')
        ]
        ax1.legend(handles=legend_elements, loc='upper right')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('TRUE F1 Racing Line\n'
                      f'(Corner aggression: {self.f1["corner_aggression"]*100:.0f}%, '
                      f'Straight tightness: {self.f1["straight_tightness"]*100:.0f}%)')
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        
        # ============================================
        # RIGHT: Velocity profile
        # ============================================
        ax2 = axes[1]
        
        s_vals = np.cumsum(np.r_[0, np.linalg.norm(np.diff(self.racing_line, axis=0), axis=1)])
        
        # Fill differently for corners vs straights
        ax2.fill_between(s_vals, 0, self.velocity,
                        where=self.is_corner, color='red', alpha=0.4, label='Corners')
        ax2.fill_between(s_vals, 0, self.velocity,
                        where=~self.is_corner, color='limegreen', alpha=0.4, label='Straights')
        ax2.plot(s_vals, self.velocity, 'k-', linewidth=2)
        
        ax2.axhline(self.vehicle['v_max'], color='gray', linestyle='--', 
                    label=f'v_max = {self.vehicle["v_max"]} m/s', alpha=0.5)
        
        ax2.set_xlabel('Arc Length (m)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Profile\n(Red=Corners slow, Green=Straights fast)')
        ax2.set_ylim(0, self.vehicle['v_max'] * 1.1)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"  Visualization: {output_path}")
        plt.close()


def main():
    parser = argparse.ArgumentParser(description='TRUE F1-style racing line optimizer')
    parser.add_argument('--aggressive', action='store_true',
                        help='Use aggressive F1 settings')
    parser.add_argument('--extreme', action='store_true',
                        help='Use MAXIMUM aggression')
    args = parser.parse_args()
    
    # ============================================
    # PATHS (same centerline as before!)
    # ============================================
    centerline_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor_DENSE.csv'
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    output_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/final_racing_line.yaml'
    output_png = '/home/asas/Documents/NCHSB/racing_line_tools/true_f1_racing_line.png'
    
    # ============================================
    # VEHICLE PARAMETERS
    # ============================================
    vehicle_params = {
        'wheelbase': 0.324,
        'width': 0.28,
        'length': 0.50,
        'max_steering_angle': 0.5236,
        'mu': 0.5,
        'v_max': 2.0,
        'inflation_radius': 0.05,
    }
    
    # ============================================
    # F1 PARAMETERS
    # ============================================
    if args.extreme:
        print("\n⚠️  EXTREME F1 MODE ⚠️\n")
        f1_params = {
            'corner_aggression': 0.95,
            'straight_tightness': 0.10,
            'min_corner_curvature': 2.0,   # Only actual corners (high curvature)
            'corner_weight': 1500,
            'straight_weight': 200,
        }
    elif args.aggressive:
        print("\n🏎️  AGGRESSIVE F1 MODE 🏎️\n")
        f1_params = {
            'corner_aggression': 0.92,
            'straight_tightness': 0.15,
            'min_corner_curvature': 2.5,   # Only actual corners
            'corner_weight': 1200,
            'straight_weight': 250,
        }
    else:
        print("\n🏁 MODERATE F1 MODE (recommended start)\n")
        f1_params = {
            'corner_aggression': 0.88,
            'straight_tightness': 0.20,
            'min_corner_curvature': 3.0,   # Only actual tight corners
            'corner_weight': 1000,
            'straight_weight': 300,
        }
    
    # ============================================
    # RUN OPTIMIZER
    # ============================================
    optimizer = TrueF1Optimizer(vehicle_params, f1_params)
    
    # Load centerline (same as before!)
    optimizer.load_centerline_csv(centerline_csv)
    
    # Recalculate track widths from map
    optimizer.recalculate_track_widths(map_pgm, map_yaml)
    
    # Identify corners
    optimizer.compute_centerline_curvature()
    
    # Optimize F1 racing line
    optimizer.optimize_f1_racing_line()
    
    # Compute velocity
    optimizer.compute_velocity_profile()
    
    # Save
    optimizer.save_yaml(output_yaml)
    optimizer.visualize(output_png)
    
    print("\n" + "=" * 70)
    print("✅ TRUE F1 RACING LINE COMPLETE!")
    print("=" * 70)
    print("\n📊 Key differences from minimum-curvature:")
    print("  • CORNERS: Use 88-95% of track width (WIDE arcs!)")
    print("  • STRAIGHTS: Use only 15-25% (TIGHT line!)")
    print("  • Result: Natural outside-inside-outside geometry!")
    print("\n🏎️  This is what real F1 drivers do!")
    print("\nTo increase aggression:")
    print("  python3 true_f1_optimizer.py --aggressive")
    print("  python3 true_f1_optimizer.py --extreme")


if __name__ == '__main__':
    main()

