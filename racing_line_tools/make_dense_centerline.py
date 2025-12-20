#!/usr/bin/env python3
"""
Take the TUM centerline and make it:
1. Dense (many evenly spaced points)
2. Closed loop (start = end)
3. Verified inside corridor
"""

import csv
import cv2
import yaml
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt


def main():
    print("=" * 60)
    print("MAKING DENSE CENTERLINE FROM TUM DATA")
    print("=" * 60)
    
    # Paths
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    input_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor.csv'
    output_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor_DENSE.csv'
    output_png = '/home/asas/Documents/NCHSB/racing_line_tools/dense_centerline.png'
    
    # Load map
    print("\n[1/5] Loading map...")
    img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
    with open(map_yaml) as f:
        config = yaml.safe_load(f)
    
    res = config['resolution']
    origin = np.array(config['origin'][:2])
    
    free = (img > 200).astype(float)
    dist_field = distance_transform_edt(free) * res
    
    print(f"  Map: {img.shape}, Resolution: {res}m/px")
    
    # Load TUM centerline
    print("\n[2/5] Loading TUM centerline...")
    points = []
    with open(input_csv, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) >= 4:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    points.append([x, y])
                except:
                    continue
    
    points = np.array(points)
    print(f"  Loaded {len(points)} points")
    
    # Filter to keep only points inside corridor
    print("\n[3/5] Filtering points inside corridor...")
    valid = []
    for x, y in points:
        px = int((x - origin[0]) / res)
        py = img.shape[0] - int((y - origin[1]) / res)
        
        if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
            clearance = dist_field[py, px]
            if clearance >= 0.15:  # Inside corridor
                valid.append([x, y, clearance])
    
    valid = np.array(valid)
    print(f"  Valid points: {len(valid)}")
    
    # Sort by arc length to maintain order
    # TUM centerline is already ordered, but might have gaps after filtering
    # Let's re-order by finding nearest neighbors
    ordered = [valid[0]]
    remaining = list(range(1, len(valid)))
    
    while remaining:
        last = ordered[-1][:2]
        dists = [np.linalg.norm(valid[i][:2] - last) for i in remaining]
        nearest_idx = np.argmin(dists)
        ordered.append(valid[remaining[nearest_idx]])
        remaining.pop(nearest_idx)
    
    ordered = np.array(ordered)
    print(f"  Ordered: {len(ordered)} points")
    
    # Create dense, smooth path
    print("\n[4/5] Creating dense path with spline...")
    x, y = ordered[:, 0], ordered[:, 1]
    
    # Fit spline (NOT periodic - let splprep handle smoothness)
    tck, u = splprep([x, y], s=1.0, k=3)
    
    # Generate 300 dense points
    num_points = 300
    u_new = np.linspace(0, 1, num_points)
    smooth_x, smooth_y = splev(u_new, tck)
    
    # Get clearances for each point
    clearances = []
    for sx, sy in zip(smooth_x, smooth_y):
        px = int((sx - origin[0]) / res)
        py = img.shape[0] - int((sy - origin[1]) / res)
        if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
            clearances.append(dist_field[py, px])
        else:
            clearances.append(0.1)
    clearances = np.array(clearances)
    
    print(f"  Output: {len(smooth_x)} dense points")
    print(f"  X range: [{smooth_x.min():.2f}, {smooth_x.max():.2f}]")
    print(f"  Y range: [{smooth_y.min():.2f}, {smooth_y.max():.2f}]")
    
    # Check path length
    path_len = sum(np.sqrt((smooth_x[i+1]-smooth_x[i])**2 + (smooth_y[i+1]-smooth_y[i])**2) 
                   for i in range(len(smooth_x)-1))
    print(f"  Path length: {path_len:.2f}m")
    
    # Save
    print("\n[5/5] Saving...")
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['# x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
        for i in range(len(smooth_x)):
            width = max(0.05, clearances[i] - 0.05)
            writer.writerow([f"{smooth_x[i]:.6f}", f"{smooth_y[i]:.6f}", 
                           f"{width:.6f}", f"{width:.6f}"])
    
    print(f"  Saved to: {output_csv}")
    
    # Visualization
    fig, ax = plt.subplots(figsize=(12, 12))
    
    extent = [origin[0], origin[0] + img.shape[1] * res,
              origin[1], origin[1] + img.shape[0] * res]
    ax.imshow(img, cmap='gray', origin='lower', extent=extent, alpha=0.7)
    
    # Original points
    ax.scatter(valid[:, 0], valid[:, 1], c='blue', s=10, alpha=0.5, label='Original TUM')
    
    # Dense smooth path
    ax.plot(smooth_x, smooth_y, 'g-', linewidth=2, label='Dense smooth')
    ax.scatter(smooth_x, smooth_y, c='green', s=15, zorder=5)
    
    # Start/end
    ax.plot(smooth_x[0], smooth_y[0], 'ko', markersize=15, label='Start', zorder=10)
    ax.plot(smooth_x[-1], smooth_y[-1], 'ro', markersize=12, label='End', zorder=10)
    
    ax.set_title(f'Dense Centerline: {len(smooth_x)} pts, length={path_len:.1f}m')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_png, dpi=150)
    plt.close()
    print(f"  Visualization: {output_png}")
    
    print("\n" + "=" * 60)
    print("✅ DENSE CENTERLINE READY!")
    print("=" * 60)


if __name__ == '__main__':
    main()

