#!/usr/bin/env python3
"""
PROPER CENTERLINE EXTRACTOR

The TUM convert_map.py produced a centerline at the map boundary with 
incorrect (too narrow) track widths.

This script extracts the TRUE geometric centerline using distance field ridges
and computes ACTUAL track widths from the distance field.
"""

import numpy as np
import yaml
import cv2
import csv
from scipy.ndimage import distance_transform_edt, maximum_filter
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from pathlib import Path


def extract_centerline(map_pgm, map_yaml, output_csv, output_png):
    """Extract proper centerline with correct track widths"""
    
    print("=" * 70)
    print("PROPER CENTERLINE EXTRACTION")
    print("=" * 70)
    
    # Load map
    print("\n[1/5] Loading map...")
    img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
    with open(map_yaml) as f:
        config = yaml.safe_load(f)
    
    resolution = config['resolution']
    origin = np.array(config['origin'][:2])
    
    print(f"  Map: {img.shape}")
    print(f"  Resolution: {resolution} m/px")
    print(f"  Origin: {origin}")
    
    # Distance transform
    print("\n[2/5] Computing distance field...")
    free_space = (img > 200).astype(float)
    distance_field = distance_transform_edt(free_space) * resolution
    
    print(f"  Max clearance: {distance_field.max():.3f}m")
    
    # Find centerline using LOCAL MAXIMA of distance field
    # These are points equidistant from walls = true geometric center
    print("\n[3/5] Finding centerline (distance field ridge)...")
    
    # Local maxima in distance field = centerline
    filter_size = 7  # Neighborhood for local max
    local_max = maximum_filter(distance_field, size=filter_size)
    
    # Ridge points: where distance field equals local max AND is significant
    min_clearance = 0.15  # At least 15cm from walls
    is_ridge = (distance_field == local_max) & (distance_field > min_clearance)
    
    # Get ridge point coordinates
    ridge_y, ridge_x = np.where(is_ridge)
    
    print(f"  Found {len(ridge_x)} ridge points")
    
    if len(ridge_x) < 10:
        print("  ERROR: Not enough ridge points found!")
        return None
    
    # Convert to world coordinates
    world_x = origin[0] + ridge_x * resolution
    world_y = origin[1] + (img.shape[0] - ridge_y) * resolution
    
    # Get clearance (half-width) at each point
    clearances = distance_field[ridge_y, ridge_x]
    
    print(f"  Clearance range: [{clearances.min():.3f}, {clearances.max():.3f}]m")
    
    # Order points to form continuous path
    print("\n[4/5] Ordering centerline points...")
    points = np.column_stack([world_x, world_y, clearances])
    
    # Sort by angle from center (creates closed loop)
    center = points[:, :2].mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    order = np.argsort(angles)
    ordered = points[order]
    
    # Subsample to ~200 points
    num_points = min(200, len(ordered))
    indices = np.linspace(0, len(ordered)-1, num_points, dtype=int)
    sampled = ordered[indices]
    
    # Smooth with spline
    print("\n[5/5] Smoothing centerline...")
    tck, u = splprep([sampled[:, 0], sampled[:, 1]], s=5.0, per=True, k=3)
    u_new = np.linspace(0, 1, 300)
    smooth_x, smooth_y = splev(u_new, tck)
    
    # Interpolate clearances
    smooth_clearances = np.interp(
        np.linspace(0, len(sampled)-1, 300),
        np.arange(len(sampled)),
        sampled[:, 2]
    )
    
    # Save to TUM format CSV
    print(f"\nSaving to: {output_csv}")
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['# x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
        
        for i in range(len(smooth_x)):
            # Use clearance as both left and right width (symmetric)
            # Subtract small safety margin
            width = max(0.1, smooth_clearances[i] - 0.05)
            writer.writerow([
                f"{smooth_x[i]:.6f}",
                f"{smooth_y[i]:.6f}",
                f"{width:.6f}",
                f"{width:.6f}"
            ])
    
    print(f"  Saved {len(smooth_x)} points")
    print(f"  Track width range: [{smooth_clearances.min():.3f}, {smooth_clearances.max():.3f}]m")
    
    # Visualization
    print(f"\nSaving visualization to: {output_png}")
    
    fig, ax = plt.subplots(figsize=(12, 12))
    
    extent = [origin[0], origin[0] + img.shape[1] * resolution,
              origin[1], origin[1] + img.shape[0] * resolution]
    ax.imshow(img, cmap='gray', origin='lower', extent=extent, alpha=0.7)
    
    # Distance field overlay
    ax.contour(distance_field, levels=10, extent=extent, colors='blue', alpha=0.3)
    
    # Centerline
    ax.plot(smooth_x, smooth_y, 'g-', linewidth=3, label='Centerline')
    scatter = ax.scatter(smooth_x, smooth_y, c=smooth_clearances, cmap='RdYlGn', 
                         s=20, zorder=5)
    plt.colorbar(scatter, ax=ax, label='Clearance (m)')
    
    # Start marker
    ax.plot(smooth_x[0], smooth_y[0], 'ro', markersize=15, label='Start')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Proper Centerline Extraction\n'
                 f'{len(smooth_x)} points, clearance: {smooth_clearances.min():.2f}-{smooth_clearances.max():.2f}m')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_png, dpi=150)
    plt.close()
    
    print("\n" + "=" * 70)
    print("✅ CENTERLINE EXTRACTION COMPLETE!")
    print("=" * 70)
    
    return smooth_x, smooth_y, smooth_clearances


def main():
    # Paths
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    output_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor_FIXED.csv'
    output_png = '/home/asas/Documents/NCHSB/racing_line_tools/proper_centerline.png'
    
    result = extract_centerline(map_pgm, map_yaml, output_csv, output_png)
    
    if result:
        print(f"\nNext step: Run true_f1_optimizer with the FIXED centerline:")
        print(f"  Edit true_f1_optimizer.py to use: {output_csv}")


if __name__ == '__main__':
    main()

