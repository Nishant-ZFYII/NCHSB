#!/usr/bin/env python3
"""
QP-based path smoothing using CVXPy.
Smooths initial racing line while respecting constraints.

From narrow_corr.pdf methodology.
"""

import numpy as np
import cvxpy as cp
import yaml
import os
import matplotlib.pyplot as plt


class QPPathSmoother:
    def __init__(self, params=None):
        """Initialize smoother with parameters"""
        self.params = params or {
            'w_smooth': 500.0,      # Reduced from 1000 - preserve curves
            'w_length': 0.5,        # Low - allow longer path for curves
            'w_deviation': 50.0,    # Allow more deviation for F1 geometry
            'max_deviation': 0.6,   # Allow bigger sweeping curves
        }
    
    def smooth(self, initial_path):
        """
        Smooth path using Quadratic Programming.
        
        Args:
            initial_path: Nx3 array of [x, y, yaw]
        
        Returns:
            smoothed_path: Nx3 array of [x, y, yaw]
        """
        N = len(initial_path)
        print(f"  Smoothing {N} waypoints...")
        
        # Decision variables: smoothed (x, y) positions
        x = cp.Variable(N)
        y = cp.Variable(N)
        
        # Build objective function
        objective = 0
        
        # 1. SMOOTHNESS: minimize second derivative (curvature)
        #    ||p[i-1] - 2*p[i] + p[i+1]||^2
        print("  Adding smoothness term...")
        for i in range(1, N-1):
            dx2 = x[i-1] - 2*x[i] + x[i+1]
            dy2 = y[i-1] - 2*y[i] + y[i+1]
            objective += self.params['w_smooth'] * (dx2**2 + dy2**2)
        
        # 2. PATH LENGTH: minimize first derivative (segment length)
        print("  Adding length term...")
        for i in range(N-1):
            dx = x[i+1] - x[i]
            dy = y[i+1] - y[i]
            objective += self.params['w_length'] * (dx**2 + dy**2)
        
        # 3. DEVIATION: stay close to initial path
        print("  Adding deviation term...")
        for i in range(N):
            dx_dev = x[i] - initial_path[i, 0]
            dy_dev = y[i] - initial_path[i, 1]
            objective += self.params['w_deviation'] * (dx_dev**2 + dy_dev**2)
        
        # Build constraints
        constraints = []
        
        # 1. Fix start and end points (for closed loop, they should match)
        print("  Adding endpoint constraints...")
        constraints.append(x[0] == initial_path[0, 0])
        constraints.append(y[0] == initial_path[0, 1])
        # For closed loop, last point connects back to first
        # We allow some deviation at the end for smooth connection
        
        # 2. Bounded deviation from initial path
        print("  Adding deviation bounds...")
        max_dev = self.params['max_deviation']
        for i in range(1, N):
            # L-infinity norm constraint (box constraint)
            constraints.append(x[i] - initial_path[i, 0] <= max_dev)
            constraints.append(x[i] - initial_path[i, 0] >= -max_dev)
            constraints.append(y[i] - initial_path[i, 1] <= max_dev)
            constraints.append(y[i] - initial_path[i, 1] >= -max_dev)
        
        # Solve QP
        print("  Solving QP...")
        problem = cp.Problem(cp.Minimize(objective), constraints)
        
        try:
            problem.solve(solver=cp.OSQP, verbose=False)
        except Exception as e:
            print(f"  OSQP failed, trying ECOS: {e}")
            problem.solve(solver=cp.ECOS, verbose=False)
        
        if problem.status not in ['optimal', 'optimal_inaccurate']:
            print(f"  Warning: QP solver status: {problem.status}")
            # Return original if optimization failed
            return initial_path.copy()
        
        print(f"  QP solved! Status: {problem.status}")
        print(f"  Optimal value: {problem.value:.4f}")
        
        # Extract solution
        smoothed_path = np.zeros_like(initial_path)
        smoothed_path[:, 0] = x.value
        smoothed_path[:, 1] = y.value
        
        # Recompute yaw from smoothed positions
        for i in range(N-1):
            dx = smoothed_path[i+1, 0] - smoothed_path[i, 0]
            dy = smoothed_path[i+1, 1] - smoothed_path[i, 1]
            smoothed_path[i, 2] = np.arctan2(dy, dx)
        
        # Last point: use direction toward first point (closed loop)
        dx = smoothed_path[0, 0] - smoothed_path[-1, 0]
        dy = smoothed_path[0, 1] - smoothed_path[-1, 1]
        smoothed_path[-1, 2] = np.arctan2(dy, dx)
        
        return smoothed_path
    
    def compute_path_stats(self, path):
        """Compute path statistics"""
        # Total length
        length = 0
        for i in range(len(path) - 1):
            dx = path[i+1, 0] - path[i, 0]
            dy = path[i+1, 1] - path[i, 1]
            length += np.sqrt(dx**2 + dy**2)
        
        # Add closing segment
        dx = path[0, 0] - path[-1, 0]
        dy = path[0, 1] - path[-1, 1]
        length += np.sqrt(dx**2 + dy**2)
        
        # Max curvature (approximate)
        max_curv = 0
        for i in range(1, len(path) - 1):
            # Second difference as curvature approximation
            dx2 = path[i-1, 0] - 2*path[i, 0] + path[i+1, 0]
            dy2 = path[i-1, 1] - 2*path[i, 1] + path[i+1, 1]
            curv = np.sqrt(dx2**2 + dy2**2)
            max_curv = max(max_curv, curv)
        
        return {
            'length': length,
            'max_curvature': max_curv,
            'num_points': len(path)
        }


def plot_comparison(initial_path, smoothed_path, output_file='smoothing_comparison.png'):
    """Plot initial vs smoothed path"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    
    # Left: Both paths overlaid
    ax1 = axes[0]
    ax1.plot(initial_path[:, 0], initial_path[:, 1], 'b.-', 
             label='Initial', alpha=0.7, markersize=4)
    ax1.plot(smoothed_path[:, 0], smoothed_path[:, 1], 'r.-', 
             label='Smoothed', alpha=0.9, markersize=4)
    
    # Draw corridor walls
    outer = 4.0
    inner = 2.0
    ax1.plot([-outer, outer, outer, -outer, -outer], 
             [-outer, -outer, outer, outer, -outer], 'k-', linewidth=2, label='Outer wall')
    ax1.plot([-inner, inner, inner, -inner, -inner], 
             [-inner, -inner, inner, inner, -inner], 'k--', linewidth=2, label='Inner wall')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Racing Line Comparison')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # Right: Deviation magnitude
    ax2 = axes[1]
    deviations = np.sqrt((smoothed_path[:, 0] - initial_path[:, 0])**2 + 
                         (smoothed_path[:, 1] - initial_path[:, 1])**2)
    ax2.plot(deviations, 'g-', linewidth=2)
    ax2.axhline(y=np.mean(deviations), color='r', linestyle='--', 
                label=f'Mean: {np.mean(deviations):.3f}m')
    ax2.axhline(y=np.max(deviations), color='orange', linestyle=':', 
                label=f'Max: {np.max(deviations):.3f}m')
    
    ax2.set_xlabel('Waypoint Index')
    ax2.set_ylabel('Deviation (m)')
    ax2.set_title('Deviation from Initial Path')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    plt.close()
    
    print(f"  Saved comparison plot to {output_file}")
    return output_file


def save_smoothed_path(path, filename='smoothed_racing_line.yaml'):
    """Save smoothed path to YAML with all required fields (s, x, y, yaw, v)"""
    waypoints = []
    s = 0.0  # Arc length
    
    for i, (x, y, yaw) in enumerate(path):
        # Compute arc length from previous point
        if i > 0:
            dx = x - path[i-1, 0]
            dy = y - path[i-1, 1]
            s += np.sqrt(dx*dx + dy*dy)
        
        # Compute speed based on curvature (slower in turns)
        if i > 0 and i < len(path) - 1:
            # Angle change as curvature proxy
            dx1 = x - path[i-1, 0]
            dy1 = y - path[i-1, 1]
            dx2 = path[i+1, 0] - x
            dy2 = path[i+1, 1] - y
            
            angle1 = np.arctan2(dy1, dx1)
            angle2 = np.arctan2(dy2, dx2)
            angle_change = abs(angle2 - angle1)
            if angle_change > np.pi:
                angle_change = 2*np.pi - angle_change
            
            # Slower for sharp turns (v between 0.3 and 0.8)
            v = max(0.3, 0.8 - angle_change * 0.5)
        else:
            v = 0.5
        
        waypoints.append({
            'index': i,
            's': float(s),
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'v': float(v)
        })
    
    data = {
        'frame_id': 'map',
        'description': 'QP-smoothed racing line',
        'num_waypoints': len(waypoints),
        'total_length': float(s),
        'waypoints': waypoints
    }
    
    with open(filename, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f"  Saved {len(waypoints)} waypoints to {filename}")
    return filename


def main():
    print("=" * 60)
    print("  QP PATH SMOOTHER - Stage 2a")
    print("=" * 60)
    
    # Find input file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, 'initial_racing_line.yaml')
    
    if not os.path.exists(input_file):
        print(f"ERROR: Input file not found: {input_file}")
        print("Please run generate_racing_line_manual.py first!")
        return
    
    # Load initial racing line
    print("\n[1/4] Loading initial racing line...")
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)
    
    waypoints = data['waypoints']
    initial_path = np.array([[wp['x'], wp['y'], wp['yaw']] for wp in waypoints])
    print(f"  Loaded {len(initial_path)} waypoints")
    
    # Initialize smoother
    print("\n[2/4] Running QP smoothing...")
    smoother = QPPathSmoother({
        'w_smooth': 1000.0,
        'w_length': 1.0,
        'w_deviation': 100.0,
        'max_deviation': 0.35,
    })
    
    smoothed_path = smoother.smooth(initial_path)
    
    # Compute statistics
    print("\n[3/4] Computing path statistics...")
    initial_stats = smoother.compute_path_stats(initial_path)
    smoothed_stats = smoother.compute_path_stats(smoothed_path)
    
    print(f"\n  Initial path:")
    print(f"    Length: {initial_stats['length']:.2f}m")
    print(f"    Max curvature: {initial_stats['max_curvature']:.4f}")
    
    print(f"\n  Smoothed path:")
    print(f"    Length: {smoothed_stats['length']:.2f}m")
    print(f"    Max curvature: {smoothed_stats['max_curvature']:.4f}")
    
    # Save outputs
    print("\n[4/4] Saving outputs...")
    output_yaml = os.path.join(script_dir, 'smoothed_racing_line.yaml')
    save_smoothed_path(smoothed_path, output_yaml)
    
    output_plot = os.path.join(script_dir, 'smoothing_comparison.png')
    plot_comparison(initial_path, smoothed_path, output_plot)
    
    print("\n" + "=" * 60)
    print("  Stage 2a Complete!")
    print("=" * 60)
    print(f"\nOutputs:")
    print(f"  - {output_yaml}")
    print(f"  - {output_plot}")
    print(f"\nNext step: Run collision_check.py")


if __name__ == '__main__':
    main()

