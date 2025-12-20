#!/usr/bin/env python3
"""
Convert TUM Raceline Optimization CSV output to YAML format for RacingLinePlanner.

Usage:
    python3 csv_to_yaml.py <input.csv> <output.yaml>

The CSV from TUM optimizer typically has columns like:
    s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
"""

import csv
import math
import yaml
import sys
import os
from pathlib import Path

def csv_to_yaml(csv_path, yaml_path):
    """Convert raceline CSV to YAML format"""
    
    waypoints = []
    
    with open(csv_path, 'r') as f:
        # Try to detect the CSV format
        first_line = f.readline()
        f.seek(0)
        
        # Check if it has a header
        if 'x' in first_line.lower() or 's_m' in first_line.lower():
            reader = csv.DictReader(f, delimiter=';' if ';' in first_line else ',')
        else:
            # No header - assume s, x, y, psi, kappa, vx, ax format
            reader = csv.DictReader(f, 
                fieldnames=['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'],
                delimiter=';' if ';' in first_line else ',')
        
        rows = list(reader)
        print(f"Loaded {len(rows)} waypoints from CSV")
        print(f"Columns: {rows[0].keys() if rows else 'none'}")
        
        for i, row in enumerate(rows):
            # Try different column name formats
            x = float(row.get('x_m') or row.get('x') or row.get('X') or 0)
            y = float(row.get('y_m') or row.get('y') or row.get('Y') or 0)
            
            # Get yaw from CSV or compute from trajectory
            yaw = None
            if 'psi_rad' in row:
                yaw = float(row['psi_rad'])
            elif 'yaw' in row:
                yaw = float(row['yaw'])
            elif 'psi' in row:
                yaw = float(row['psi'])
            
            # Compute yaw from forward difference if not provided
            if yaw is None and i < len(rows) - 1:
                next_row = rows[i + 1]
                nx = float(next_row.get('x_m') or next_row.get('x') or 0)
                ny = float(next_row.get('y_m') or next_row.get('y') or 0)
                yaw = math.atan2(ny - y, nx - x)
            elif yaw is None:
                yaw = waypoints[-1]['yaw'] if waypoints else 0
            
            # Get arc length
            s = float(row.get('s_m') or row.get('s') or i * 0.1)
            
            # Get velocity (default to 1.0 if not available)
            v = float(row.get('vx_mps') or row.get('v') or row.get('velocity') or 1.0)
            
            waypoints.append({
                'index': i,
                's': float(s),
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw),
                'v': float(v)
            })
    
    # Compute total length
    total_length = waypoints[-1]['s'] if waypoints else 0
    
    data = {
        'description': 'Raceline from TUM Minimum-Time Optimizer',
        'frame_id': 'map',
        'num_waypoints': len(waypoints),
        'total_length': total_length,
        'waypoints': waypoints
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f"\nSaved {len(waypoints)} waypoints to {yaml_path}")
    print(f"Total length: {total_length:.2f}m")
    
    # Print sample waypoints
    print("\nSample waypoints:")
    for i in [0, len(waypoints)//4, len(waypoints)//2, 3*len(waypoints)//4]:
        wp = waypoints[i]
        print(f"  [{i}]: x={wp['x']:.2f}, y={wp['y']:.2f}, v={wp['v']:.2f}")

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 csv_to_yaml.py <input.csv> <output.yaml>")
        print("\nExample:")
        print("  python3 csv_to_yaml.py ~/Documents/Raceline-Optimization/outputs/traj_race_cl.csv final_racing_line.yaml")
        sys.exit(1)
    
    csv_path = Path(sys.argv[1])
    yaml_path = Path(sys.argv[2])
    
    if not csv_path.exists():
        print(f"Error: Input file not found: {csv_path}")
        sys.exit(1)
    
    print(f"Converting: {csv_path}")
    print(f"Output: {yaml_path}")
    
    csv_to_yaml(csv_path, yaml_path)
    print("\nDone!")

if __name__ == '__main__':
    main()

