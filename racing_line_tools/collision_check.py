#!/usr/bin/env python3
"""
Collision detection using distance field.
SIMPLIFIED approach - no complex OBB polygon intersection!

Uses scipy.ndimage.distance_transform_edt for efficient distance computation.
"""

import cv2
import numpy as np
import yaml
import os
from scipy.ndimage import distance_transform_edt


class CollisionChecker:
    def __init__(self, map_pgm, map_yaml, method='footprint'):
        """
        Initialize collision checker.
        
        Args:
            map_pgm: Path to map image file
            map_yaml: Path to map YAML config
            method: 'circle' or 'footprint'
                - 'circle': Approximate robot as circle (faster)
                - 'footprint': Sample rectangular footprint points (more accurate)
        """
        # Load map
        print("  Loading map...")
        self.map_img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise ValueError(f"Could not load map from {map_pgm}")
        
        with open(map_yaml, 'r') as f:
            config = yaml.safe_load(f)
        
        self.resolution = config['resolution']
        self.origin = np.array(config['origin'][:2])
        
        # Compute distance field
        print("  Computing distance field...")
        # Binary map: free=1, occupied=0
        # Threshold: values > 250 are free (white), others are occupied
        binary_map = (self.map_img > 250).astype(float)
        
        # Distance transform gives distance to nearest occupied cell
        distance_pixels = distance_transform_edt(binary_map)
        self.distance_field = distance_pixels * self.resolution
        
        print(f"  Distance field shape: {self.distance_field.shape}")
        print(f"  Max clearance: {np.max(self.distance_field):.3f}m")
        
        self.method = method
        
        # Robot footprint sample points (from robot specs)
        # Footprint: [[0.155, 0.11], [0.155, -0.11], [-0.155, -0.11], [-0.155, 0.11]]
        self.footprint_points = [
            ( 0.155,  0.11),   # Front-right
            ( 0.155, -0.11),   # Front-left
            (-0.155,  0.11),   # Rear-right
            (-0.155, -0.11),   # Rear-left
            ( 0.0,    0.11),   # Mid-right
            ( 0.0,   -0.11),   # Mid-left
            ( 0.155,  0.0),    # Front-center
            (-0.155,  0.0),    # Rear-center
        ]
        
        # Robot dimensions
        self.robot_radius = 0.18  # Circumscribed radius for circle method
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy
    
    def get_clearance(self, x, y):
        """Get clearance at world coordinate"""
        gx, gy = self.world_to_grid(x, y)
        
        # Check bounds
        if gx < 0 or gy < 0 or \
           gx >= self.distance_field.shape[1] or \
           gy >= self.distance_field.shape[0]:
            return 0.0  # Out of map = no clearance
        
        return self.distance_field[gy, gx]
    
    def check_collision_circle(self, waypoint, margin=0.05):
        """
        Check collision using circle approximation.
        
        Args:
            waypoint: [x, y, yaw]
            margin: Additional safety margin (meters)
        
        Returns:
            (has_collision, clearance): Tuple of bool and float
        """
        x, y = waypoint[0], waypoint[1]
        clearance = self.get_clearance(x, y)
        
        required_clearance = self.robot_radius + margin
        has_collision = clearance < required_clearance
        
        return has_collision, clearance
    
    def check_collision_footprint(self, waypoint, margin=0.05):
        """
        Check collision using sampled footprint points.
        More accurate for rectangular robot.
        
        Args:
            waypoint: [x, y, yaw]
            margin: Safety margin (meters)
        
        Returns:
            (has_collision, min_clearance): Tuple of bool and float
        """
        x, y, yaw = waypoint
        
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        min_clearance = float('inf')
        
        # Check each footprint point
        for fx, fy in self.footprint_points:
            # Transform to world frame
            world_x = x + fx * cos_yaw - fy * sin_yaw
            world_y = y + fx * sin_yaw + fy * cos_yaw
            
            clearance = self.get_clearance(world_x, world_y)
            min_clearance = min(min_clearance, clearance)
            
            # Early exit if collision
            if clearance < margin:
                return True, clearance
        
        has_collision = min_clearance < margin
        return has_collision, min_clearance
    
    def check_path_collisions(self, path, margin=0.05):
        """
        Check entire path for collisions.
        
        Returns:
            collision_info: Dict with collision indices and clearances
        """
        collision_indices = []
        clearances = []
        
        for i, waypoint in enumerate(path):
            if self.method == 'circle':
                has_collision, clearance = self.check_collision_circle(waypoint, margin)
            else:
                has_collision, clearance = self.check_collision_footprint(waypoint, margin)
            
            clearances.append(clearance)
            
            if has_collision:
                collision_indices.append(i)
        
        return {
            'collision_indices': collision_indices,
            'clearances': np.array(clearances),
            'min_clearance': np.min(clearances),
            'mean_clearance': np.mean(clearances),
            'num_collisions': len(collision_indices)
        }
    
    def visualize_clearances(self, path, output_file='clearance_visualization.png'):
        """Visualize path with clearance coloring"""
        import matplotlib.pyplot as plt
        
        # Check collisions
        info = self.check_path_collisions(path)
        clearances = info['clearances']
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Plot path colored by clearance
        scatter = ax.scatter(path[:, 0], path[:, 1], c=clearances, 
                            cmap='RdYlGn', s=20, vmin=0, vmax=0.5)
        plt.colorbar(scatter, label='Clearance (m)')
        
        # Mark collision points
        if info['collision_indices']:
            coll_pts = path[info['collision_indices']]
            ax.scatter(coll_pts[:, 0], coll_pts[:, 1], c='red', s=100, 
                      marker='x', linewidths=2, label='Collision')
        
        # Draw corridor walls
        outer = 4.0
        inner = 2.0
        ax.plot([-outer, outer, outer, -outer, -outer], 
                [-outer, -outer, outer, outer, -outer], 'k-', linewidth=2)
        ax.plot([-inner, inner, inner, -inner, -inner], 
                [-inner, -inner, inner, inner, -inner], 'k--', linewidth=2)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Path Clearances (Min: {info["min_clearance"]:.3f}m, Collisions: {info["num_collisions"]})')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(output_file, dpi=150)
        plt.close()
        
        print(f"  Saved clearance visualization to {output_file}")
        return output_file


def add_arclength_and_speed(path, base_speed=1.5, min_speed=0.5):
    """
    Add arc-length and speed profile to path.
    Speed is reduced at high curvature points.
    """
    N = len(path)
    result = []
    s = 0.0
    
    for i in range(N):
        x, y, yaw = path[i]
        
        # Compute arc-length
        if i > 0:
            dx = x - path[i-1, 0]
            dy = y - path[i-1, 1]
            ds = np.sqrt(dx**2 + dy**2)
            s += ds
        
        # Compute local curvature (for speed profile)
        if 0 < i < N-1:
            dx2 = path[i-1, 0] - 2*x + path[i+1, 0]
            dy2 = path[i-1, 1] - 2*y + path[i+1, 1]
            curvature = np.sqrt(dx2**2 + dy2**2)
        else:
            curvature = 0.0
        
        # Speed profile: slower at high curvature
        # v = base_speed * exp(-k * curvature)
        speed = max(min_speed, base_speed * np.exp(-10 * curvature))
        
        result.append({
            's': float(s),
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'v': float(speed)
        })
    
    return result


def save_final_racing_line(waypoints, filename='final_racing_line.yaml'):
    """Save final racing line with all attributes"""
    data = {
        'frame_id': 'map',
        'description': 'Final collision-free racing line with speed profile',
        'num_waypoints': len(waypoints),
        'total_length': waypoints[-1]['s'] if waypoints else 0.0,
        'waypoints': waypoints
    }
    
    with open(filename, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f"  Saved {len(waypoints)} waypoints to {filename}")
    return filename


def main():
    print("=" * 60)
    print("  COLLISION CHECK - Stage 2b")
    print("=" * 60)
    
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    input_file = os.path.join(script_dir, 'smoothed_racing_line.yaml')
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    
    if not os.path.exists(input_file):
        print(f"ERROR: Input file not found: {input_file}")
        print("Please run qp_smooth_path.py first!")
        return
    
    # Load smoothed path
    print("\n[1/4] Loading smoothed racing line...")
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)
    
    waypoints = data['waypoints']
    path = np.array([[wp['x'], wp['y'], wp['yaw']] for wp in waypoints])
    print(f"  Loaded {len(path)} waypoints")
    
    # Initialize collision checker
    print("\n[2/4] Initializing collision checker...")
    checker = CollisionChecker(map_pgm, map_yaml, method='footprint')
    
    # Check collisions
    print("\n[3/4] Checking for collisions...")
    margin = 0.05  # 5cm safety margin
    info = checker.check_path_collisions(path, margin)
    
    print(f"\n  Collision check results:")
    print(f"    Method: {checker.method}")
    print(f"    Safety margin: {margin}m")
    print(f"    Collision points: {info['num_collisions']}")
    print(f"    Min clearance: {info['min_clearance']:.3f}m")
    print(f"    Mean clearance: {info['mean_clearance']:.3f}m")
    
    if info['num_collisions'] > 0:
        print(f"\n  ⚠️  WARNING: {info['num_collisions']} collision points detected!")
        print(f"    Collision indices: {info['collision_indices'][:10]}...")  # First 10
        print(f"\n  Recommendation: Increase safety margin or adjust racing line")
    else:
        print(f"\n  ✓ Path is COLLISION-FREE!")
    
    # Visualize
    vis_file = os.path.join(script_dir, 'clearance_visualization.png')
    checker.visualize_clearances(path, vis_file)
    
    # Add arc-length and speed profile
    print("\n[4/4] Adding speed profile and saving...")
    final_waypoints = add_arclength_and_speed(path, base_speed=1.5, min_speed=0.5)
    
    output_file = os.path.join(script_dir, 'final_racing_line.yaml')
    save_final_racing_line(final_waypoints, output_file)
    
    # Also copy to maps folder for easy access
    maps_output = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/final_racing_line.yaml'
    save_final_racing_line(final_waypoints, maps_output)
    
    print("\n" + "=" * 60)
    print("  Stage 2b Complete!")
    print("=" * 60)
    print(f"\nOutputs:")
    print(f"  - {output_file}")
    print(f"  - {maps_output}")
    print(f"  - {vis_file}")
    print(f"\nTotal path length: {final_waypoints[-1]['s']:.2f}m")
    print(f"\nNext step: Build Nav2 plugin (Stage 3)")


if __name__ == '__main__':
    main()

