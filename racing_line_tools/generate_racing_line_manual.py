#!/usr/bin/env python3
"""
FIXED F1-Style Racing Line Generator

Creates aggressive outside-inside-outside curves like real F1 racing.
More waypoints + better interpolation = smoother curves.
"""

import numpy as np
import yaml
import cv2
import matplotlib.pyplot as plt
from pathlib import Path

class F1RacingLineGenerator:
    def __init__(self, map_pgm, map_yaml):
        # Load map
        with open(map_yaml, 'r') as f:
            self.map_config = yaml.safe_load(f)
        
        self.resolution = self.map_config['resolution']
        self.origin = np.array(self.map_config['origin'][:2])
        self.map_img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
        
        # Corridor walls (adjust these to match your map analysis)
        self.west_left = -3.15
        self.west_right = -2.05
        self.east_left = 3.55
        self.east_right = 4.45
        self.north_top = 0.47
        self.north_bottom = -0.98
        self.south_top = -6.03
        self.south_bottom = -7.48
        
        # Racing parameters
        self.wall_clearance = 0.25  # Stay 25cm from walls (safer)
        self.apex_tightness = 0.85  # How much to cut apex (0.5=center, 1.0=full inside)
        
        print(f"F1 Racing Line Generator initialized")
        print(f"Corridor dimensions:")
        print(f"  West: {self.west_left:.2f} to {self.west_right:.2f}")
        print(f"  East: {self.east_left:.2f} to {self.east_right:.2f}")
        print(f"  North: {self.north_bottom:.2f} to {self.north_top:.2f}")
        print(f"  South: {self.south_bottom:.2f} to {self.south_top:.2f}")
    
    def lerp(self, a, b, t):
        """Linear interpolation"""
        return a + t * (b - a)
    
    def create_f1_waypoints(self):
        """
        Create F1-style racing line with outside-inside-outside geometry.
        
        Counter-clockwise around square:
        South (west-bound) → SW corner → West (north-bound) → NW corner → 
        North (east-bound) → NE corner → East (south-bound) → SE corner → repeat
        """
        
        waypoints = []
        
        # Safe positions in each corridor
        west_outer = self.west_left + self.wall_clearance    # -2.90
        west_inner = self.west_right - self.wall_clearance   # -2.30
        west_center = (west_outer + west_inner) / 2
        
        east_outer = self.east_right - self.wall_clearance   # 4.20
        east_inner = self.east_left + self.wall_clearance    # 3.80
        east_center = (east_outer + east_inner) / 2
        
        north_outer = self.north_top - self.wall_clearance   # 0.22
        north_inner = self.north_bottom + self.wall_clearance  # -0.73
        north_center = (north_outer + north_inner) / 2
        
        south_outer = self.south_bottom + self.wall_clearance  # -7.23
        south_inner = self.south_top - self.wall_clearance   # -6.28
        south_center = (south_outer + south_inner) / 2
        
        print(f"\nRacing line positions:")
        print(f"  West corridor: outer={west_outer:.2f}, inner={west_inner:.2f}")
        print(f"  East corridor: outer={east_outer:.2f}, inner={east_inner:.2f}")
        print(f"  North corridor: outer={north_outer:.2f}, inner={north_inner:.2f}")
        print(f"  South corridor: outer={south_outer:.2f}, inner={south_inner:.2f}")
        
        # =================================================================
        # SOUTH CORRIDOR (heading WEST: from east side to west side)
        # LEFT TURN ahead (SW corner), so hug RIGHT side (bottom = south_outer)
        # =================================================================
        print("\n=== South Corridor (heading west) ===")
        
        # Straight section before SW corner - hug OUTER (bottom) wall
        waypoints.append({'x': 3.5, 'y': south_outer, 'desc': 'South start - outer'})
        waypoints.append({'x': 2.5, 'y': south_outer, 'desc': 'South straight'})
        waypoints.append({'x': 1.5, 'y': south_outer, 'desc': 'South straight'})
        waypoints.append({'x': 0.5, 'y': south_outer, 'desc': 'South straight'})
        waypoints.append({'x': -0.5, 'y': south_outer, 'desc': 'Approaching SW'})
        
        # =================================================================
        # SW CORNER (LEFT turn: west-bound → north-bound)
        # Entry: OUTER (bottom), Apex: INNER (top-right), Exit: OUTER (left)
        # =================================================================
        print("=== SW Corner (left turn) ===")
        
        # Entry phase: still on outer wall, start turning
        waypoints.append({'x': -1.2, 'y': south_outer, 'desc': 'SW entry - wide'})
        waypoints.append({'x': -1.7, 'y': south_outer + 0.15, 'desc': 'SW entry - start curve'})
        
        # Turn-in: move toward apex
        waypoints.append({'x': -2.1, 'y': south_center, 'desc': 'SW turn-in'})
        
        # APEX: Cut to inside (top of south corridor = inner)
        apex_x = west_inner  # -2.30 (right side of west corridor)
        apex_y = self.lerp(south_inner, north_center, 0.3)  # toward top
        waypoints.append({'x': apex_x, 'y': apex_y, 'desc': 'SW APEX - inside'})
        
        # Exit: drift to outer (left wall of west corridor)
        waypoints.append({'x': west_center, 'y': -5.5, 'desc': 'SW exit'})
        waypoints.append({'x': west_outer, 'y': -5.0, 'desc': 'SW exit - wide'})
        
        # =================================================================
        # WEST CORRIDOR (heading NORTH: from south to north)
        # LEFT TURN ahead (NW corner), so hug RIGHT side (left = west_outer)
        # =================================================================
        print("=== West Corridor (heading north) ===")
        
        # Straight section - hug OUTER (left) wall
        waypoints.append({'x': west_outer, 'y': -4.0, 'desc': 'West straight'})
        waypoints.append({'x': west_outer, 'y': -3.0, 'desc': 'West straight'})
        waypoints.append({'x': west_outer, 'y': -2.0, 'desc': 'West straight'})
        waypoints.append({'x': west_outer, 'y': -1.2, 'desc': 'Approaching NW'})
        
        # =================================================================
        # NW CORNER (LEFT turn: north-bound → east-bound)
        # Entry: OUTER (left), Apex: INNER (bottom-right), Exit: OUTER (top)
        # =================================================================
        print("=== NW Corner (left turn) ===")
        
        # Entry phase
        waypoints.append({'x': west_outer, 'y': -0.6, 'desc': 'NW entry - wide'})
        waypoints.append({'x': west_outer - 0.1, 'y': -0.2, 'desc': 'NW entry - start curve'})
        
        # Turn-in
        waypoints.append({'x': west_center, 'y': north_center, 'desc': 'NW turn-in'})
        
        # APEX: Cut to inside
        apex_x = self.lerp(west_inner, east_center, 0.25)  # moving right
        apex_y = north_inner  # bottom of north corridor
        waypoints.append({'x': apex_x, 'y': apex_y, 'desc': 'NW APEX - inside'})
        
        # Exit
        waypoints.append({'x': -1.0, 'y': north_center, 'desc': 'NW exit'})
        waypoints.append({'x': -0.3, 'y': north_outer, 'desc': 'NW exit - wide'})
        
        # =================================================================
        # NORTH CORRIDOR (heading EAST: from west to east)
        # LEFT TURN ahead (NE corner), so hug RIGHT side (top = north_outer)
        # =================================================================
        print("=== North Corridor (heading east) ===")
        
        # Straight section - hug OUTER (top) wall
        waypoints.append({'x': 0.5, 'y': north_outer, 'desc': 'North straight'})
        waypoints.append({'x': 1.5, 'y': north_outer, 'desc': 'North straight'})
        waypoints.append({'x': 2.5, 'y': north_outer, 'desc': 'North straight'})
        waypoints.append({'x': 3.0, 'y': north_outer, 'desc': 'Approaching NE'})
        
        # =================================================================
        # NE CORNER (LEFT turn: east-bound → south-bound)
        # Entry: OUTER (top), Apex: INNER (bottom-left), Exit: OUTER (right)
        # =================================================================
        print("=== NE Corner (left turn) ===")
        
        # Entry phase
        waypoints.append({'x': 3.4, 'y': north_outer, 'desc': 'NE entry - wide'})
        waypoints.append({'x': 3.6, 'y': north_outer - 0.15, 'desc': 'NE entry - start curve'})
        
        # Turn-in
        waypoints.append({'x': east_center, 'y': north_center, 'desc': 'NE turn-in'})
        
        # APEX: Cut to inside
        apex_x = east_inner  # left side of east corridor
        apex_y = self.lerp(north_inner, south_center, 0.3)  # moving down
        waypoints.append({'x': apex_x, 'y': apex_y, 'desc': 'NE APEX - inside'})
        
        # Exit
        waypoints.append({'x': east_center, 'y': -1.5, 'desc': 'NE exit'})
        waypoints.append({'x': east_outer, 'y': -2.0, 'desc': 'NE exit - wide'})
        
        # =================================================================
        # EAST CORRIDOR (heading SOUTH: from north to south)
        # LEFT TURN ahead (SE corner), so hug RIGHT side (right = east_outer)
        # =================================================================
        print("=== East Corridor (heading south) ===")
        
        # Straight section - hug OUTER (right) wall
        waypoints.append({'x': east_outer, 'y': -3.0, 'desc': 'East straight'})
        waypoints.append({'x': east_outer, 'y': -4.0, 'desc': 'East straight'})
        waypoints.append({'x': east_outer, 'y': -5.0, 'desc': 'East straight'})
        waypoints.append({'x': east_outer, 'y': -5.8, 'desc': 'Approaching SE'})
        
        # =================================================================
        # SE CORNER (LEFT turn: south-bound → west-bound)
        # Entry: OUTER (right), Apex: INNER (top-left), Exit: OUTER (bottom)
        # =================================================================
        print("=== SE Corner (left turn) ===")
        
        # Entry phase
        waypoints.append({'x': east_outer, 'y': -6.2, 'desc': 'SE entry - wide'})
        waypoints.append({'x': east_outer - 0.1, 'y': -6.5, 'desc': 'SE entry - start curve'})
        
        # Turn-in
        waypoints.append({'x': east_center, 'y': south_center, 'desc': 'SE turn-in'})
        
        # APEX: Cut to inside
        apex_x = self.lerp(east_inner, west_center, 0.25)  # moving left
        apex_y = south_inner  # top of south corridor
        waypoints.append({'x': apex_x, 'y': apex_y, 'desc': 'SE APEX - inside'})
        
        # Exit - connect back to start
        waypoints.append({'x': 2.0, 'y': south_center, 'desc': 'SE exit'})
        waypoints.append({'x': 1.0, 'y': south_outer, 'desc': 'SE exit - wide'})
        
        # Close the loop
        waypoints.append({'x': 0.0, 'y': south_outer, 'desc': 'Loop closing'})
        
        print(f"\nCreated {len(waypoints)} base waypoints")
        return waypoints
    
    def interpolate_smooth(self, waypoints, points_per_meter=25):
        """
        Create smooth interpolation between waypoints.
        More points = smoother curves in visualization.
        """
        
        interpolated = []
        
        for i in range(len(waypoints)):
            wp1 = waypoints[i]
            wp2 = waypoints[(i + 1) % len(waypoints)]
            
            # Distance between waypoints
            dx = wp2['x'] - wp1['x']
            dy = wp2['y'] - wp1['y']
            dist = np.sqrt(dx**2 + dy**2)
            
            # Number of interpolation points based on distance
            num_points = max(3, int(dist * points_per_meter))
            
            for t in np.linspace(0, 1, num_points, endpoint=False):
                x = self.lerp(wp1['x'], wp2['x'], t)
                y = self.lerp(wp1['y'], wp2['y'], t)
                
                # Compute yaw from direction
                if i < len(waypoints) - 1:
                    yaw = np.arctan2(dy, dx)
                else:
                    yaw = interpolated[-1]['yaw'] if interpolated else 0
                
                interpolated.append({'x': x, 'y': y, 'yaw': yaw})
        
        print(f"Interpolated to {len(interpolated)} points ({points_per_meter} pts/meter)")
        return interpolated
    
    def compute_path_metrics(self, waypoints):
        """Add arclength (s) and speed (v) to waypoints"""
        result = []
        s = 0.0
        
        for i, wp in enumerate(waypoints):
            if i > 0:
                prev = waypoints[i-1]
                ds = np.sqrt((wp['x'] - prev['x'])**2 + (wp['y'] - prev['y'])**2)
                s += ds
            
            # Simple constant speed for now
            v = 1.5  # m/s
            
            result.append({
                'index': i,
                's': float(s),
                'x': float(wp['x']),
                'y': float(wp['y']),
                'yaw': float(wp['yaw']),
                'v': float(v)
            })
        
        return result
    
    def visualize_on_map(self, waypoints, output_file='f1_racing_line_preview.png'):
        """Draw racing line on map"""
        
        map_color = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        def world_to_pixel(x, y):
            px = int((x - self.origin[0]) / self.resolution)
            py = int((y - self.origin[1]) / self.resolution)
            py = self.map_img.shape[0] - py
            return px, py
        
        # Draw path
        for i in range(len(waypoints) - 1):
            pt1 = world_to_pixel(waypoints[i]['x'], waypoints[i]['y'])
            pt2 = world_to_pixel(waypoints[i+1]['x'], waypoints[i+1]['y'])
            cv2.line(map_color, pt1, pt2, (0, 255, 0), 3)  # Green, thicker
        
        # Draw start
        start_pt = world_to_pixel(waypoints[0]['x'], waypoints[0]['y'])
        cv2.circle(map_color, start_pt, 10, (0, 0, 255), -1)
        
        cv2.imwrite(output_file, map_color)
        print(f"Saved visualization to {output_file}")
    
    def save_yaml(self, waypoints, filename='initial_racing_line.yaml'):
        """Save waypoints to YAML"""
        
        data = {
            'frame_id': 'map',
            'is_closed_loop': True,
            'num_waypoints': len(waypoints),
            'total_length': waypoints[-1]['s'] if waypoints else 0,
            'waypoints': waypoints
        }
        
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        # Stats
        total_length = waypoints[-1]['s'] if waypoints else 0
        print(f"\nSaved to {filename}")
        print(f"  Waypoints: {len(waypoints)}")
        print(f"  Total length: {total_length:.2f} m")

def main():
    # Find map
    script_dir = Path(__file__).parent
    map_dir = script_dir.parent / 'src' / 'rc_model_description' / 'maps'
    if not map_dir.exists():
        map_dir = Path('/home/asas/Documents/NCHSB/src/rc_model_description/maps')
    
    map_pgm = str(map_dir / 'narrow_corridor.pgm')
    map_yaml = str(map_dir / 'narrow_corridor.yaml')
    
    print("=" * 70)
    print("F1-STYLE RACING LINE GENERATOR (FIXED)")
    print("=" * 70)
    
    generator = F1RacingLineGenerator(map_pgm, map_yaml)
    
    # Create base waypoints with F1 geometry
    print("\n[1/4] Creating F1-style waypoints...")
    base_waypoints = generator.create_f1_waypoints()
    
    # Interpolate for smooth curves
    print("\n[2/4] Interpolating for smooth curves...")
    smooth_waypoints = generator.interpolate_smooth(base_waypoints, points_per_meter=25)
    
    # Add metrics
    print("\n[3/4] Computing path metrics...")
    final_waypoints = generator.compute_path_metrics(smooth_waypoints)
    
    # Visualize
    print("\n[4/4] Creating visualization...")
    generator.visualize_on_map(smooth_waypoints, str(script_dir / 'f1_racing_line_preview.png'))
    
    # Save
    generator.save_yaml(final_waypoints, str(script_dir / 'initial_racing_line.yaml'))
    
    # Also save directly as final (skip QP if desired)
    generator.save_yaml(final_waypoints, str(map_dir / 'final_racing_line.yaml'))
    
    print("\n" + "=" * 70)
    print("✓ F1 RACING LINE GENERATED!")
    print("=" * 70)
    print("\nNext steps:")
    print("  1. Check f1_racing_line_preview.png - should see CURVES!")
    print("  2. Optionally run: python3 qp_smooth_path.py")
    print("  3. Run: python3 collision_check.py")
    print()

if __name__ == '__main__':
    main()
