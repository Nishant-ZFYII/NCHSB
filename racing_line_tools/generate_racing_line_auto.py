#!/usr/bin/env python3
"""
Automatically generate racing line from map by extracting centerline.
Uses skeletonization to find the corridor center, then applies racing offsets.
"""

import numpy as np
import yaml
import cv2
import os
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize
from collections import deque


class AutoRacingLineGenerator:
    def __init__(self, map_pgm_path, map_yaml_path):
        """Load map and compute distance field"""
        # Load map
        print("  Loading map...")
        self.map_img = cv2.imread(map_pgm_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise ValueError(f"Could not load map from {map_pgm_path}")
        
        with open(map_yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.resolution = config['resolution']
        self.origin = np.array(config['origin'][:2])
        
        print(f"  Map shape: {self.map_img.shape}")
        print(f"  Resolution: {self.resolution}m/pixel")
        print(f"  Origin: {self.origin}")
        
        # Binary map: free=1, occupied=0
        self.binary_map = (self.map_img > 250).astype(np.uint8)
        
        # Compute distance field (distance to nearest wall)
        print("  Computing distance field...")
        self.distance_field = distance_transform_edt(self.binary_map) * self.resolution
        
    def pixel_to_world(self, px, py):
        """Convert pixel coordinates to world coordinates"""
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + py * self.resolution
        return wx, wy
    
    def world_to_pixel(self, wx, wy):
        """Convert world coordinates to pixel coordinates"""
        px = int((wx - self.origin[0]) / self.resolution)
        py = int((wy - self.origin[1]) / self.resolution)
        return px, py
    
    def extract_centerline(self):
        """Extract corridor centerline using skeletonization"""
        print("  Extracting centerline via skeletonization...")
        
        # Skeletonize the free space
        skeleton = skeletonize(self.binary_map > 0)
        
        # Get skeleton points
        skeleton_points = np.argwhere(skeleton)
        print(f"  Found {len(skeleton_points)} skeleton pixels")
        
        if len(skeleton_points) == 0:
            raise ValueError("No skeleton points found!")
        
        # Order the skeleton points by traversing from one end
        ordered = self.order_skeleton_points(skeleton, skeleton_points)
        
        return ordered
    
    def order_skeleton_points(self, skeleton, points):
        """Order skeleton points by graph traversal"""
        if len(points) == 0:
            return np.array([])
        
        # Build adjacency for skeleton
        h, w = skeleton.shape
        point_set = set(map(tuple, points))
        
        # Find endpoints (points with only 1 neighbor)
        endpoints = []
        for py, px in points:
            neighbors = 0
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = py + dy, px + dx
                    if (ny, nx) in point_set:
                        neighbors += 1
            if neighbors == 1:
                endpoints.append((py, px))
        
        if not endpoints:
            # No clear endpoints, pick point with max distance from walls
            max_dist_idx = np.argmax([self.distance_field[p[0], p[1]] for p in points])
            start = tuple(points[max_dist_idx])
        else:
            start = endpoints[0]
        
        # BFS traversal from start
        visited = set()
        ordered = []
        queue = deque([start])
        
        while queue:
            current = queue.popleft()
            if current in visited:
                continue
            
            visited.add(current)
            ordered.append(current)
            
            py, px = current
            # Find unvisited neighbors (8-connectivity)
            neighbors = []
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = py + dy, px + dx
                    if (ny, nx) in point_set and (ny, nx) not in visited:
                        neighbors.append((ny, nx))
            
            # Sort neighbors by distance to current to get smooth ordering
            neighbors.sort(key=lambda n: abs(n[0]-py) + abs(n[1]-px))
            for n in neighbors:
                queue.append(n)
        
        return np.array(ordered)
    
    def sample_centerline(self, centerline_pixels, spacing_m=0.3):
        """Sample centerline at regular intervals"""
        if len(centerline_pixels) == 0:
            return np.array([])
        
        # Convert to world coordinates
        world_points = []
        for py, px in centerline_pixels:
            wx, wy = self.pixel_to_world(px, py)
            world_points.append([wx, wy])
        world_points = np.array(world_points)
        
        # Compute cumulative arc length
        diffs = np.diff(world_points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        arc_lengths = np.concatenate([[0], np.cumsum(segment_lengths)])
        total_length = arc_lengths[-1]
        
        print(f"  Total centerline length: {total_length:.2f}m")
        
        # Sample at regular intervals
        num_samples = max(10, int(total_length / spacing_m))
        sample_s = np.linspace(0, total_length, num_samples)
        
        sampled = []
        for s in sample_s:
            # Find the segment containing this arc length
            idx = np.searchsorted(arc_lengths, s) - 1
            idx = max(0, min(idx, len(world_points) - 2))
            
            # Interpolate within segment
            t = (s - arc_lengths[idx]) / (arc_lengths[idx + 1] - arc_lengths[idx] + 1e-6)
            t = min(1.0, max(0.0, t))
            
            pt = world_points[idx] + t * (world_points[idx + 1] - world_points[idx])
            sampled.append(pt)
        
        return np.array(sampled)
    
    def compute_curvature(self, points, window=3):
        """Compute curvature at each point"""
        n = len(points)
        curvatures = np.zeros(n)
        
        for i in range(window, n - window):
            # Use points before and after
            p_prev = points[i - window]
            p_curr = points[i]
            p_next = points[i + window]
            
            # Vectors
            v1 = p_curr - p_prev
            v2 = p_next - p_curr
            
            # Cross product (signed curvature)
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            
            # Normalize by arc length
            len1 = np.linalg.norm(v1)
            len2 = np.linalg.norm(v2)
            if len1 > 0 and len2 > 0:
                curvatures[i] = cross / (len1 * len2)
        
        return curvatures
    
    def apply_racing_offset(self, centerline, curvatures, max_offset=0.3):
        """
        Apply lateral offset based on curvature to create racing line.
        
        For LEFT turns (positive curvature): offset RIGHT before, LEFT at apex
        For RIGHT turns (negative curvature): offset LEFT before, RIGHT at apex
        """
        n = len(centerline)
        racing_line = centerline.copy()
        
        # Smooth curvatures
        from scipy.ndimage import gaussian_filter1d
        smooth_curv = gaussian_filter1d(curvatures, sigma=3)
        
        # Compute normals (perpendicular to path direction)
        normals = np.zeros_like(centerline)
        for i in range(n - 1):
            tangent = centerline[i + 1] - centerline[i]
            tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
            # Normal is perpendicular (rotate 90 degrees)
            normals[i] = np.array([-tangent[1], tangent[0]])
        normals[-1] = normals[-2]
        
        # Apply offset based on curvature
        # Positive curvature = left turn, offset to the right (positive normal)
        # Negative curvature = right turn, offset to the left (negative normal)
        for i in range(n):
            # Racing line offset: opposite to curvature direction (outside of turn)
            # But at apex, we want to be on inside
            # For simplicity: offset proportional to curvature, towards outside
            offset = -smooth_curv[i] * max_offset * 5  # Scale factor
            offset = np.clip(offset, -max_offset, max_offset)
            
            racing_line[i] = centerline[i] + offset * normals[i]
        
        return racing_line
    
    def add_yaw(self, points):
        """Add yaw (heading) to points"""
        n = len(points)
        path = np.zeros((n, 3))
        path[:, :2] = points
        
        for i in range(n - 1):
            dx = points[i + 1, 0] - points[i, 0]
            dy = points[i + 1, 1] - points[i, 1]
            path[i, 2] = np.arctan2(dy, dx)
        
        # Last point uses previous yaw
        path[-1, 2] = path[-2, 2]
        
        return path
    
    def check_clearances(self, points, min_clearance=0.15):
        """Check that all points have sufficient clearance from walls"""
        clearances = []
        violations = []
        
        for i, (x, y) in enumerate(points):
            px, py = self.world_to_pixel(x, y)
            
            # Check bounds
            if 0 <= px < self.distance_field.shape[1] and \
               0 <= py < self.distance_field.shape[0]:
                clearance = self.distance_field[py, px]
            else:
                clearance = 0.0
            
            clearances.append(clearance)
            if clearance < min_clearance:
                violations.append(i)
        
        return np.array(clearances), violations
    
    def save_path(self, path, filename):
        """Save path to YAML file"""
        waypoints = []
        s = 0.0
        
        for i, (x, y, yaw) in enumerate(path):
            if i > 0:
                dx = x - path[i-1, 0]
                dy = y - path[i-1, 1]
                s += np.sqrt(dx**2 + dy**2)
            
            waypoints.append({
                'index': i,
                's': float(s),
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw),
                'v': 1.0  # Default speed
            })
        
        data = {
            'frame_id': 'map',
            'description': 'Auto-generated racing line from map centerline',
            'num_waypoints': len(waypoints),
            'total_length': float(s),
            'waypoints': waypoints
        }
        
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        print(f"  Saved {len(waypoints)} waypoints to {filename}")
    
    def visualize(self, centerline, racing_line, output_file):
        """Visualize centerline and racing line on map"""
        vis = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        # Draw centerline (blue)
        for x, y in centerline:
            px, py = self.world_to_pixel(x, y)
            if 0 <= px < vis.shape[1] and 0 <= py < vis.shape[0]:
                cv2.circle(vis, (px, py), 2, (255, 0, 0), -1)
        
        # Draw racing line (green)
        for i in range(len(racing_line) - 1):
            x1, y1 = racing_line[i, 0], racing_line[i, 1]
            x2, y2 = racing_line[i+1, 0], racing_line[i+1, 1]
            
            px1, py1 = self.world_to_pixel(x1, y1)
            px2, py2 = self.world_to_pixel(x2, y2)
            
            cv2.line(vis, (px1, py1), (px2, py2), (0, 255, 0), 2)
        
        # Mark start (red circle)
        px, py = self.world_to_pixel(racing_line[0, 0], racing_line[0, 1])
        cv2.circle(vis, (px, py), 6, (0, 0, 255), -1)
        
        cv2.imwrite(output_file, vis)
        print(f"  Saved visualization to {output_file}")


def main():
    print("=" * 60)
    print("  AUTO RACING LINE GENERATOR")
    print("=" * 60)
    
    # Paths
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Initialize generator
    print("\n[1/5] Loading map and computing distance field...")
    generator = AutoRacingLineGenerator(map_pgm, map_yaml)
    
    # Extract centerline
    print("\n[2/5] Extracting centerline...")
    centerline_pixels = generator.extract_centerline()
    
    # Sample at regular intervals
    print("\n[3/5] Sampling centerline...")
    centerline = generator.sample_centerline(centerline_pixels, spacing_m=0.25)
    print(f"  Sampled {len(centerline)} points")
    
    # Compute curvature and apply racing offset
    print("\n[4/5] Computing racing line...")
    curvatures = generator.compute_curvature(centerline)
    racing_points = generator.apply_racing_offset(centerline, curvatures, max_offset=0.25)
    
    # Check clearances and adjust if needed
    clearances, violations = generator.check_clearances(racing_points, min_clearance=0.12)
    print(f"  Min clearance: {clearances.min():.3f}m")
    print(f"  Mean clearance: {clearances.mean():.3f}m")
    
    if violations:
        print(f"  ⚠️  {len(violations)} points below min clearance, adjusting...")
        # Pull points back toward centerline for low-clearance areas
        for i in violations:
            t = 0.5  # Blend 50% back to centerline
            racing_points[i] = t * centerline[i] + (1-t) * racing_points[i]
    
    # Add yaw
    racing_path = generator.add_yaw(racing_points)
    
    # Check clearances again
    clearances, violations = generator.check_clearances(racing_path[:, :2], min_clearance=0.1)
    print(f"  Final min clearance: {clearances.min():.3f}m")
    print(f"  Final violations: {len(violations)}")
    
    # Save outputs
    print("\n[5/5] Saving outputs...")
    
    # Save as initial racing line (for compatibility with existing pipeline)
    initial_file = os.path.join(script_dir, 'initial_racing_line.yaml')
    generator.save_path(racing_path, initial_file)
    
    # Also save as final (skip QP smoothing since we're already smooth)
    final_file = os.path.join(script_dir, 'final_racing_line.yaml')
    generator.save_path(racing_path, final_file)
    
    # Copy to maps folder
    maps_file = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/final_racing_line.yaml'
    generator.save_path(racing_path, maps_file)
    
    # Visualize
    vis_file = os.path.join(script_dir, 'auto_racing_line.png')
    generator.visualize(centerline, racing_path, vis_file)
    
    print("\n" + "=" * 60)
    print("  AUTO GENERATION COMPLETE!")
    print("=" * 60)
    print(f"\nOutputs:")
    print(f"  - {initial_file}")
    print(f"  - {final_file}")
    print(f"  - {maps_file}")
    print(f"  - {vis_file}")


if __name__ == '__main__':
    main()

