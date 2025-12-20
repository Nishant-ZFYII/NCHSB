#!/usr/bin/env python3
"""
Navigation Results Recorder and Plotter
Records bag data during navigation and generates presentation-ready plots.

Usage:
  1. Start your system (Gazebo, SLAM, Nav2)
  2. Run: python3 record_and_plot.py record
  3. Send Nav2 goal and let robot navigate
  4. Press Ctrl+C to stop recording
  5. Run: python3 record_and_plot.py plot <bag_folder>
"""

import subprocess
import sys
import os
from datetime import datetime

# Bag output directory
RESULTS_DIR = os.path.dirname(os.path.abspath(__file__))

# Topics to record
TOPICS = [
    '/odom',                                    # Actual position/velocity
    '/cmd_vel',                                 # Commanded velocity
    '/ackermann_steering_controller/reference', # Stamped velocity command
    '/plan',                                    # Nav2 planned path
    '/tf',                                      # Transforms
]

def record():
    """Start recording bag file"""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name = f'nav_run_{timestamp}'
    bag_path = os.path.join(RESULTS_DIR, bag_name)
    
    print("=" * 60)
    print("NAV2 RESULTS RECORDER")
    print("=" * 60)
    print(f"\nRecording to: {bag_path}")
    print(f"Topics: {', '.join(TOPICS)}")
    print("\n>>> Send your Nav2 goal now! <<<")
    print(">>> Press Ctrl+C when navigation completes <<<\n")
    
    cmd = ['ros2', 'bag', 'record', '-o', bag_path] + TOPICS
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print(f"\n\nRecording stopped!")
        print(f"Bag saved to: {bag_path}")
        print(f"\nTo generate plots, run:")
        print(f"  python3 {__file__} plot {bag_name}")

def plot(bag_folder):
    """Generate plots from recorded bag"""
    import sqlite3
    import numpy as np
    import matplotlib.pyplot as plt
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, TwistStamped
    
    bag_path = os.path.join(RESULTS_DIR, bag_folder)
    db_path = os.path.join(bag_path, f'{bag_folder}_0.db3')
    
    if not os.path.exists(db_path):
        # Try mcap format
        mcap_path = os.path.join(bag_path, f'{bag_folder}_0.mcap')
        if os.path.exists(mcap_path):
            print("MCAP format detected. Converting with ros2 bag...")
            print("For now, please use PlotJuggler:")
            print(f"  ros2 run plotjuggler plotjuggler")
            print(f"  Then load: {mcap_path}")
            return
        print(f"Bag not found: {db_path}")
        return
    
    print(f"Loading bag: {db_path}")
    
    # Extract data from SQLite bag
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get topic IDs
    cursor.execute("SELECT id, name FROM topics")
    topic_map = {name: id for id, name in cursor.fetchall()}
    
    # Extract odometry data
    times = []
    x_pos = []
    y_pos = []
    velocities = []
    
    if '/odom' in topic_map:
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ?",
            (topic_map['/odom'],)
        )
        for timestamp, data in cursor.fetchall():
            msg = deserialize_message(data, Odometry)
            times.append(timestamp / 1e9)  # Convert to seconds
            x_pos.append(msg.pose.pose.position.x)
            y_pos.append(msg.pose.pose.position.y)
            velocities.append(msg.twist.twist.linear.x)
    
    conn.close()
    
    if not times:
        print("No odometry data found!")
        return
    
    # Normalize time to start at 0
    t0 = times[0]
    times = [t - t0 for t in times]
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Navigation Results: {bag_folder}', fontsize=14, fontweight='bold')
    
    # Plot 1: Trajectory (X-Y)
    ax1 = axes[0, 0]
    ax1.plot(x_pos, y_pos, 'b-', linewidth=2, label='Actual Path')
    ax1.scatter(x_pos[0], y_pos[0], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(x_pos[-1], y_pos[-1], c='red', s=100, marker='x', label='End', zorder=5)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Robot Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Velocity over time
    ax2 = axes[0, 1]
    ax2.plot(times, velocities, 'b-', linewidth=1.5)
    ax2.axhline(y=np.mean(velocities), color='r', linestyle='--', label=f'Avg: {np.mean(velocities):.2f} m/s')
    ax2.axhline(y=np.max(velocities), color='g', linestyle=':', label=f'Max: {np.max(velocities):.2f} m/s')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity Profile')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Velocity along path (colored trajectory)
    ax3 = axes[1, 0]
    scatter = ax3.scatter(x_pos, y_pos, c=velocities, cmap='RdYlGn', s=10)
    plt.colorbar(scatter, ax=ax3, label='Velocity (m/s)')
    ax3.set_xlabel('X Position (m)')
    ax3.set_ylabel('Y Position (m)')
    ax3.set_title('Velocity Along Path (Green=Fast, Red=Slow)')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # Plot 4: Statistics
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    # Calculate statistics
    total_time = times[-1]
    total_distance = sum(np.sqrt((x_pos[i+1]-x_pos[i])**2 + (y_pos[i+1]-y_pos[i])**2) 
                        for i in range(len(x_pos)-1))
    avg_vel = np.mean(velocities)
    max_vel = np.max(velocities)
    min_vel = np.min([v for v in velocities if v > 0.01])  # Exclude near-zero
    
    stats_text = f"""
    NAVIGATION STATISTICS
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    Total Time:        {total_time:.2f} s
    Total Distance:    {total_distance:.2f} m
    
    Average Velocity:  {avg_vel:.2f} m/s
    Maximum Velocity:  {max_vel:.2f} m/s
    Minimum Velocity:  {min_vel:.2f} m/s
    
    Start Position:    ({x_pos[0]:.2f}, {y_pos[0]:.2f})
    End Position:      ({x_pos[-1]:.2f}, {y_pos[-1]:.2f})
    """
    
    ax4.text(0.1, 0.5, stats_text, fontsize=12, fontfamily='monospace',
             verticalalignment='center', transform=ax4.transAxes,
             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    
    # Save plot
    output_file = os.path.join(RESULTS_DIR, f'{bag_folder}_results.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_file}")
    
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 record_and_plot.py record          # Start recording")
        print("  python3 record_and_plot.py plot <bag_name> # Generate plots")
        print("")
        print("Or use PlotJuggler directly:")
        print("  ros2 run plotjuggler plotjuggler")
        return
    
    command = sys.argv[1]
    
    if command == 'record':
        record()
    elif command == 'plot':
        if len(sys.argv) < 3:
            # List available bags
            print("Available bags:")
            for f in os.listdir(RESULTS_DIR):
                if os.path.isdir(os.path.join(RESULTS_DIR, f)) and f.startswith('nav_run'):
                    print(f"  {f}")
            print("\nUsage: python3 record_and_plot.py plot <bag_name>")
        else:
            plot(sys.argv[2])
    else:
        print(f"Unknown command: {command}")

if __name__ == '__main__':
    main()

