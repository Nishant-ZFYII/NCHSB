#!/usr/bin/env bash
#
# Full corridor ablation pipeline for db3 bags:
#   Phase 1 — Build a SLAM Toolbox map from the LiDAR bag
#   Phase 2 — Run 4 costmap configs (L, L+S, L+D, D) sequentially
#
# Usage:
#   bash run_corridor_ablation_db3.sh
#
set -euo pipefail

export PATH="/usr/bin:/usr/local/bin:$PATH"
source /opt/ros/humble/setup.bash
source "$HOME/nchsb_ws/install/setup.bash"

# ---------- configuration ----------
BAG_DB3="$HOME/rosbags/rgbd_imu_20260302_172822/rgbd_imu_20260302_172822_0.db3"
BAG_DUR=263   # seconds
MAP_DIR="$HOME/maps"
MAP_NAME="corridor_172822"
MAP_PG="${MAP_DIR}/${MAP_NAME}_posegraph"
OUTPUT_DIR="${MAP_DIR}/corridor_ablation_results"

CONFIGS=("L" "L+S" "L+D" "D")

HW_PKG=$(ros2 pkg prefix rc_hardware_bringup --share)
URDF="${HW_PKG}/urdf/rc_model_replay.urdf.xacro"
SLAM_MAP_YAML="${HW_PKG}/config/slam_hardware.yaml"
SLAM_LOC_YAML="${HW_PKG}/config/slam_localization_hardware.yaml"
NAV2_BASE="${HW_PKG}/config/nav2_hardware.yaml"

mkdir -p "$MAP_DIR" "$OUTPUT_DIR"

cleanup_all() {
    echo "[cleanup] Killing lingering ROS nodes..."
    pkill -9 -f "db3_replayer" 2>/dev/null || true
    pkill -9 -f "slam_toolbox" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*odom.*base_link" 2>/dev/null || true
    pkill -9 -f "costmap_evaluator" 2>/dev/null || true
    pkill -9 -f "da3_inference" 2>/dev/null || true
    pkill -9 -f "da3_to_pointcloud" 2>/dev/null || true
    pkill -9 -f "sensor_depth_to_pointcloud" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "waypoint_follower" 2>/dev/null || true
    pkill -9 -f "velocity_smoother" 2>/dev/null || true
    pkill -9 -f "lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "navigation_launch" 2>/dev/null || true
    sleep 3
}

# ===================================================================
# PHASE 1: BUILD MAP
# ===================================================================
echo "============================================="
echo "PHASE 1: BUILDING STATIC MAP"
echo "  Bag: $BAG_DB3"
echo "  Duration: ${BAG_DUR}s"
echo "============================================="

cleanup_all

ROBOT_DESC=$(xacro "$URDF")

# Start TF tree
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p "robot_description:=$ROBOT_DESC" -p use_sim_time:=false &
RSP_PID=$!

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 odom base_link &
TF_PID=$!

sleep 2

# Start SLAM Toolbox in mapping mode
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$SLAM_MAP_YAML" \
    -p use_sim_time:=false &
SLAM_PID=$!

sleep 3

# Replay the bag at 1x speed for mapping
ros2 run rc_hardware_bringup db3_replayer.py --ros-args \
    -p "bag_path:=$BAG_DB3" \
    -p rate:=1.0 \
    -p loop:=false &
REPLAY_PID=$!

WAIT_MAP=$((BAG_DUR + 30))
echo "  Waiting up to ${WAIT_MAP}s for mapping to complete..."
sleep "$WAIT_MAP"

# Save the map
echo "  Saving map..."
ros2 run nav2_map_server map_saver_cli \
    -f "${MAP_DIR}/${MAP_NAME}" --ros-args -p use_sim_time:=false 2>&1 || true
sleep 2

# Save posegraph for localization
echo "  Saving posegraph..."
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${MAP_PG}'}" 2>&1 || true
sleep 3

echo "  Map saved to: ${MAP_DIR}/${MAP_NAME}.pgm"
echo "  Posegraph saved to: ${MAP_PG}.posegraph"

cleanup_all

# ===================================================================
# PHASE 2: COSTMAP ABLATION
# ===================================================================
echo ""
echo "============================================="
echo "PHASE 2: COSTMAP ABLATION"
echo "  Configs: ${CONFIGS[*]}"
echo "  Map: ${MAP_PG}"
echo "============================================="

for CFG in "${CONFIGS[@]}"; do
    echo ""
    echo ">>> Starting config: $CFG  ($(date))"

    RUN_RATE="1.0"
    if [[ "$CFG" == "L+D" || "$CFG" == "D" ]]; then
        RUN_RATE="0.3"
        echo "    (DA3 config — rate=${RUN_RATE}x)"
    fi

    cleanup_all

    CFG_DIR="${OUTPUT_DIR}/${CFG}"
    mkdir -p "$CFG_DIR"

    # Launch the full ablation stack via the existing launch file
    ros2 launch rc_hardware_bringup costmap_ablation.launch.py \
        "costmap_config:=$CFG" \
        "bag_path:=$BAG_DB3" \
        "map_posegraph:=$MAP_PG" \
        "rate:=$RUN_RATE" &
    LAUNCH_PID=$!

    sleep 15

    # Start costmap evaluator
    ros2 run rc_hardware_bringup costmap_evaluator.py --ros-args \
        -p "config_label:=$CFG" \
        -p "output_dir:=$OUTPUT_DIR" \
        -p snapshot_interval:=5.0 &
    EVAL_PID=$!

    WAIT_SECS=$(python3 -c "import math; print(int(math.ceil($BAG_DUR / $RUN_RATE)) + 30)")
    echo "    Waiting ${WAIT_SECS}s for playback..."
    sleep "$WAIT_SECS"

    echo "    Shutting down config: $CFG"
    kill $EVAL_PID 2>/dev/null || true
    sleep 2
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 3

    cleanup_all

    echo "    Done: $CFG — results in $CFG_DIR/"
done

echo ""
echo "============================================="
echo "ALL PHASES COMPLETE"
echo "============================================="

# Print summaries
for CFG in "${CONFIGS[@]}"; do
    SUMMARY="$OUTPUT_DIR/$CFG/summary.json"
    if [ -f "$SUMMARY" ]; then
        echo ""
        echo "--- $CFG ---"
        cat "$SUMMARY"
    fi
done
