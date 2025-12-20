#!/usr/bin/env python3
"""
Fix the TUM centerline by filtering out points outside the corridor
and regenerating the racing line.
"""

import csv
import cv2
import yaml
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import splprep, splev

def main():
    print("=" * 60)
    print("FIXING CENTERLINE - Remove points outside corridor")
    print("=" * 60)
    
    # Paths
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    input_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor.csv'
    output_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor_INSIDE.csv'
    
    # Load map
    print("\n[1/4] Loading map...")
    img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
    with open(map_yaml) as f:
        config = yaml.safe_load(f)
    
    res = config['resolution']
    origin = np.array(config['origin'][:2])
    
    # Distance field
    free = (img > 200).astype(float)
    dist_field = distance_transform_edt(free) * res
    
    print(f"  Map: {img.shape}")
    print(f"  Origin: {origin}")
    print(f"  Resolution: {res}")
    
    # Load TUM centerline
    print("\n[2/4] Loading TUM centerline...")
    points = []
    with open(input_csv, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0].startswith('#'):
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                wr = float(row[2])
                wl = float(row[3])
                points.append([x, y, wr, wl])
            except:
                continue
    
    points = np.array(points)
    print(f"  Loaded {len(points)} points")
    print(f"  X range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}]")
    print(f"  Y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}]")
    
    # Filter points inside corridor (clearance > min_clearance)
    print("\n[3/4] Filtering points inside corridor...")
    min_clearance = 0.15  # At least 15cm from wall
    
    valid_points = []
    for p in points:
        x, y, wr, wl = p
        
        # Convert to pixel coordinates
        px = int((x - origin[0]) / res)
        py = img.shape[0] - int((y - origin[1]) / res)
        
        # Check bounds
        if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
            clearance = dist_field[py, px]
            if clearance >= min_clearance:
                valid_points.append(p)
    
    valid_points = np.array(valid_points)
    print(f"  Valid points: {len(valid_points)} / {len(points)}")
    print(f"  Removed {len(points) - len(valid_points)} points outside corridor")
    
    if len(valid_points) < 10:
        print("  ERROR: Not enough valid points!")
        return
    
    print(f"  New X range: [{valid_points[:,0].min():.2f}, {valid_points[:,0].max():.2f}]")
    print(f"  New Y range: [{valid_points[:,1].min():.2f}, {valid_points[:,1].max():.2f}]")
    
    # Recalculate track widths from distance field
    print("\n[4/4] Recalculating track widths...")
    for i, p in enumerate(valid_points):
        x, y = p[0], p[1]
        px = int((x - origin[0]) / res)
        py = img.shape[0] - int((y - origin[1]) / res)
        
        if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
            clearance = dist_field[py, px]
            # Use clearance as symmetric width, minus safety margin
            width = max(0.05, clearance - 0.05)
            valid_points[i, 2] = width  # wr
            valid_points[i, 3] = width  # wl
    
    # Save fixed centerline
    print(f"\nSaving to: {output_csv}")
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['# x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
        for p in valid_points:
            writer.writerow([f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", f"{p[3]:.6f}"])
    
    print(f"  Saved {len(valid_points)} points")
    print(f"  Track width range: [{valid_points[:,2].min():.3f}, {valid_points[:,2].max():.3f}]m")
    
    print("\n" + "=" * 60)
    print("✅ CENTERLINE FIXED!")
    print("=" * 60)
    print(f"\nNext: Update true_f1_optimizer.py to use:")
    print(f"  {output_csv}")


if __name__ == '__main__':
    main()

