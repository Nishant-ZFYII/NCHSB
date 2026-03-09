#!/usr/bin/env bash
# Build static map from bag 172822 (has LiDAR)
set -eo pipefail

export PATH="/usr/bin:/usr/local/bin:$PATH"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/nchsb_ws/install/setup.bash" 2>/dev/null || true

BAG="$HOME/rosbags/rgbd_imu_20260302_172822/rgbd_imu_20260302_172822_0.db3"
MAP_DIR="$HOME/maps"
MAP_NAME="corridor_172822"
MAP_PG="${MAP_DIR}/${MAP_NAME}_posegraph"

HW_PKG=$(ros2 pkg prefix rc_hardware_bringup --share)
URDF="${HW_PKG}/urdf/rc_model_replay.urdf.xacro"
SLAM_YAML="${HW_PKG}/config/slam_hardware.yaml"

mkdir -p "$MAP_DIR"

echo "=== MAPPING: bag 172822 (263s) ==="

ROBOT_DESC=$(xacro "$URDF")

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p "robot_description:=$ROBOT_DESC" -p use_sim_time:=false &

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 odom base_link &

sleep 2

ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$SLAM_YAML" -p use_sim_time:=false &

sleep 3

echo "  Starting bag replay at 1x..."
ros2 run rc_hardware_bringup db3_replayer.py --ros-args \
    -p "bag_path:=$BAG" -p rate:=1.0 -p loop:=false &

# Wait for bag to complete (263s + overhead)
echo "  Sleeping 300s for replay completion..."
sleep 300

echo "  Saving map..."
ros2 run nav2_map_server map_saver_cli \
    -f "${MAP_DIR}/${MAP_NAME}" --ros-args -p use_sim_time:=false 2>&1 || echo "map_saver failed (non-fatal)"
sleep 3

echo "  Saving posegraph..."
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${MAP_PG}'}" 2>&1 || echo "serialize failed (non-fatal)"
sleep 3

echo "  Map: ${MAP_DIR}/${MAP_NAME}.pgm"
echo "  Posegraph: ${MAP_PG}.posegraph"

# Cleanup
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "static_transform_publisher.*odom" 2>/dev/null || true
pkill -9 -f "db3_replayer" 2>/dev/null || true
sleep 2

echo "=== MAPPING COMPLETE ==="
