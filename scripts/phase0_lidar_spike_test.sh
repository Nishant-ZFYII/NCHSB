#!/bin/bash
# ============================================================================
# Phase 0: LiDAR-Actor Visibility Spike Test (Self-contained)
#
# This test bypasses the full robot launch (no ros2_control, no xacro).
# It uses a minimal world with an embedded gpu_lidar sensor on a static box.
#
# Prerequisites:
#   sudo apt-get install ros-humble-ros-gz
#   cd ~/MS_Project/NCHSB && colcon build --symlink-install
#   source install/setup.bash
#
# Test geometry (robot at 1.0, 0.0 facing +X):
#   RED static cylinder:   (3.0, 0.0) = 2m ahead   -> bearing ~0 deg
#   BLUE dynamic cylinder: (1.0, 2.0) = 2m left     -> bearing ~90 deg
#   South wall:            y = -2                    -> bearing ~-90 deg
#   North wall:            y = +2                    -> bearing ~+90 deg
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================"
echo "  Phase 0: LiDAR-Actor Visibility Spike"
echo "  (Self-contained, no ros2_control needed)"
echo "============================================"
echo ""

# Source ROS + workspace
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

echo "[1/4] Checking prerequisites..."
if ! ros2 pkg list 2>/dev/null | grep -q "ros_gz_sim"; then
    echo "ERROR: ros_gz_sim not found."
    echo "Install: sudo apt-get install ros-humble-ros-gz"
    exit 1
fi
echo "  ros_gz_sim:    OK"

if ! ros2 pkg list 2>/dev/null | grep -q "ros_gz_bridge"; then
    echo "ERROR: ros_gz_bridge not found."
    exit 1
fi
echo "  ros_gz_bridge: OK"
echo ""

# Resolve world file path
WORLD_FILE="$WORKSPACE_DIR/src/rc_model_description/worlds/lidar_actor_spike_test.world"
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found: $WORLD_FILE"
    exit 1
fi
echo "[2/4] World file: $WORLD_FILE"
echo ""

echo "[3/4] Starting Gazebo Ignition Fortress..."
echo "  (This may take 15-30 seconds to initialize)"
echo ""

# Launch Gazebo in background
ign gazebo "$WORLD_FILE" -r &
GZ_PID=$!
echo "  Gazebo PID: $GZ_PID"

# Wait for Gazebo to start
sleep 8

echo ""
echo "[4/4] Starting scan bridge..."
echo ""

# Bridge the LiDAR topic
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &
BRIDGE_PID=$!

echo "  Bridge PID: $BRIDGE_PID"
echo ""
echo "============================================"
echo "  Test is running!"
echo ""
echo "  In another terminal, run the analyzer:"
echo "    cd ~/MS_Project/NCHSB"
echo "    source /opt/ros/humble/setup.bash"
echo "    python3 scripts/phase0_analyze_scan.py"
echo ""
echo "  Or manually check:"
echo "    ros2 topic echo /scan --once"
echo ""
echo "  Press Ctrl+C to stop everything."
echo "============================================"

# Trap Ctrl+C to kill both processes
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $BRIDGE_PID 2>/dev/null
    kill $GZ_PID 2>/dev/null
    wait $BRIDGE_PID 2>/dev/null
    wait $GZ_PID 2>/dev/null
    echo "Done."
}
trap cleanup SIGINT SIGTERM

# Wait for Gazebo to exit
wait $GZ_PID
