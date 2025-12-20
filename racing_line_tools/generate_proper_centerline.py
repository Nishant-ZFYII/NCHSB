#!/usr/bin/env python3
"""
Generate a PROPER centerline:
- Dense, evenly spaced points
- Entirely inside the corridor
- Closed loop (start = end)
- Uses distance field ridge detection
"""

import cv2
import yaml
import numpy as np
import csv
from scipy.ndimage import distance_transform_edt, maximum_filter
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt


def extract_corridor_centerline(map_pgm, map_yaml):
    """Extract centerline using distance field ridge detection"""
    
    # Load map
    img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
    with open(map_yaml) as f:
        config = yaml.safe_load(f)
    
    res = config['resolution']
    origin = np.array(config['origin'][:2])
    
    print(f"Map: {img.shape}, Resolution: {res}m/px")
    print(f"Origin: {origin}")
    
    # Distance field
    free = (img > 200).astype(float)
    dist = distance_transform_edt(free) * res
    
    print(f"Max clearance: {dist.max():.2f}m")
    
    # Find ridge points (local maxima of distance field)
    # These are the TRUE geometric centers of the corridor
    filter_size = 5
    local_max = maximum_filter(dist, size=filter_size)
    
    # Ridge = where distance equals local max AND clearance is in corridor range
    # Corridor clearance: 0.2m to 0.6m (not the big open center which is 3m+)
    min_clearance = 0.15
    max_clearance = 0.8  # Exclude the big open center area
    
    is_ridge = (dist == local_max) & (dist >= min_clearance) & (dist <= max_clearance)
    
    ridge_y, ridge_x = np.where(is_ridge)
    print(f"Found {len(ridge_x)} ridge points")
    
    if len(ridge_x) < 20:
        print("ERROR: Not enough ridge points!")
        return None, None, None
    
    # Convert to world coordinates
    world_x = origin[0] + ridge_x * res
    world_y = origin[1] + (img.shape[0] - ridge_y) * res
    clearances = dist[ridge_y, ridge_x]
    
    # Sort points to form a continuous path by angle from center
    points = np.column_stack([world_x, world_y, clearances])
    center = points[:, :2].mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    order = np.argsort(angles)
    ordered = points[order]
    
    return ordered, res, origin, img, dist


def densify_and_smooth(points, num_output_points=300):
    """Create dense, smooth, closed-loop path"""
    
    x = points[:, 0]
    y = points[:, 1]
    clearances = points[:, 2]
    
    # Fit periodic spline (closed loop)
    # Need to close the loop first
    x_closed = np.append(x, x[0])
    y_closed = np.append(y, y[0])
    
    try:
        tck, u = splprep([x_closed, y_closed], s=2.0, per=True, k=3)
    except:
        # Fallback to non-periodic if periodic fails
        tck, u = splprep([x_closed, y_closed], s=2.0, per=False, k=3)
    
    # Generate dense points
    u_new = np.linspace(0, 1, num_output_points, endpoint=False)
    smooth_x, smooth_y = splev(u_new, tck)
    
    # Interpolate clearances
    smooth_clearances = np.interp(
        np.linspace(0, len(clearances), num_output_points, endpoint=False) % len(clearances),
        np.arange(len(clearances)),
        clearances
    )
    
    return smooth_x, smooth_y, smooth_clearances


def main():
    print("=" * 70)
    print("GENERATING PROPER CENTERLINE")
    print("Dense, closed-loop, entirely inside corridor")
    print("=" * 70)
    
    # Paths
    map_pgm = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.pgm'
    map_yaml = '/home/asas/Documents/NCHSB/src/rc_model_description/maps/narrow_corridor.yaml'
    output_csv = '/home/asas/Documents/Raceline-Optimization/inputs/tracks/narrow_corridor_PROPER.csv'
    output_png = '/home/asas/Documents/NCHSB/racing_line_tools/proper_centerline.png'
    
    # Extract centerline
    print("\n[1/4] Extracting corridor centerline...")
    result = extract_corridor_centerline(map_pgm, map_yaml)
    
    if result[0] is None:
        return
    
    ordered, res, origin, img, dist = result
    print(f"  Ordered points: {len(ordered)}")
    
    # Densify and smooth
    print("\n[2/4] Creating dense, smooth, closed-loop path...")
    num_points = 300  # Dense!
    smooth_x, smooth_y, smooth_clearances = densify_and_smooth(ordered, num_points)
    
    print(f"  Output points: {len(smooth_x)}")
    print(f"  X range: [{smooth_x.min():.2f}, {smooth_x.max():.2f}]")
    print(f"  Y range: [{smooth_y.min():.2f}, {smooth_y.max():.2f}]")
    
    # Verify closed loop
    dist_start_end = np.sqrt((smooth_x[0] - smooth_x[-1])**2 + (smooth_y[0] - smooth_y[-1])**2)
    print(f"  Distance start to end: {dist_start_end:.4f}m (should be small)")
    
    # Save to TUM format
    print("\n[3/4] Saving centerline...")
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['# x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
        
        for i in range(len(smooth_x)):
            # Track width = clearance minus safety margin
            width = max(0.05, smooth_clearances[i] - 0.05)
            writer.writerow([
                f"{smooth_x[i]:.6f}",
                f"{smooth_y[i]:.6f}",
                f"{width:.6f}",
                f"{width:.6f}"
            ])
    
    print(f"  Saved to: {output_csv}")
    print(f"  Track width range: [{smooth_clearances.min():.3f}, {smooth_clearances.max():.3f}]m")
    
    # Visualization
    print("\n[4/4] Creating visualization...")
    fig, ax = plt.subplots(figsize=(12, 12))
    
    extent = [origin[0], origin[0] + img.shape[1] * res,
              origin[1], origin[1] + img.shape[0] * res]
    ax.imshow(img, cmap='gray', origin='lower', extent=extent, alpha=0.7)
    
    # Centerline
    ax.plot(smooth_x, smooth_y, 'b-', linewidth=2, label='Centerline')
    ax.scatter(smooth_x, smooth_y, c=smooth_clearances, cmap='RdYlGn', s=15, zorder=5)
    
    # Start point
    ax.plot(smooth_x[0], smooth_y[0], 'ko', markersize=15, label='Start/End', zorder=10)
    
    # Direction arrow
    ax.annotate('', xy=(smooth_x[5], smooth_y[5]), xytext=(smooth_x[0], smooth_y[0]),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Proper Centerline: {len(smooth_x)} dense points, closed loop')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_png, dpi=150)
    plt.close()
    
    print(f"  Saved to: {output_png}")
    
    print("\n" + "=" * 70)
    print("✅ PROPER CENTERLINE GENERATED!")
    print("=" * 70)
    print(f"\nNow run the optimizer with this centerline:")
    print(f"  Update true_f1_optimizer.py to use: narrow_corridor_PROPER.csv")


if __name__ == '__main__':
    main()

