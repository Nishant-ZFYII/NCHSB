#!/usr/bin/env bash
# Run 4-config costmap ablation on bag 172822
set -eo pipefail

export PATH="/usr/bin:/usr/local/bin:$PATH"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/nchsb_ws/install/setup.bash" 2>/dev/null || true

BAG="$HOME/rosbags/rgbd_imu_20260302_172822/rgbd_imu_20260302_172822_0.db3"
BAG_DUR=263
MAP_PG="$HOME/maps/corridor_172822_posegraph"
OUTPUT_DIR="$HOME/maps/corridor_ablation_results"

CONFIGS=("L" "L+S" "L+D" "D")

mkdir -p "$OUTPUT_DIR"

cleanup_all() {
    pkill -9 -f "db3_replayer" 2>/dev/null || true
    pkill -9 -f "mcap_replayer" 2>/dev/null || true
    pkill -9 -f "bag_replayer" 2>/dev/null || true
    pkill -9 -f "slam_toolbox" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
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
    sleep 4
}

echo "============================================="
echo "CORRIDOR COSTMAP ABLATION (bag 172822)"
echo "  Bag: $BAG (${BAG_DUR}s)"
echo "  Map: $MAP_PG"
echo "  Output: $OUTPUT_DIR"
echo "  Configs: ${CONFIGS[*]}"
echo "============================================="

for CFG in "${CONFIGS[@]}"; do
    echo ""
    echo ">>> CONFIG: $CFG  — $(date)"

    cleanup_all

    RUN_RATE="1.0"
    if [[ "$CFG" == "L+D" || "$CFG" == "D" ]]; then
        RUN_RATE="0.3"
    fi
    echo "    Rate: ${RUN_RATE}x"

    mkdir -p "${OUTPUT_DIR}/${CFG}"

    ros2 launch rc_hardware_bringup costmap_ablation.launch.py \
        "costmap_config:=$CFG" \
        "bag_path:=$BAG" \
        "map_posegraph:=$MAP_PG" \
        "rate:=$RUN_RATE" &
    LAUNCH_PID=$!

    sleep 15

    ros2 run rc_hardware_bringup costmap_evaluator.py --ros-args \
        -p "config_label:=$CFG" \
        -p "output_dir:=$OUTPUT_DIR" \
        -p snapshot_interval:=5.0 &
    EVAL_PID=$!

    WAIT=$(python3 -c "import math; print(int(math.ceil($BAG_DUR / $RUN_RATE)) + 40)")
    echo "    Waiting ${WAIT}s..."
    sleep "$WAIT"

    echo "    Done: $CFG"
    kill $EVAL_PID 2>/dev/null || true
    sleep 2
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 3
    cleanup_all
done

echo ""
echo "============================================="
echo "ABLATION COMPLETE"
echo "============================================="

for CFG in "${CONFIGS[@]}"; do
    SUMMARY="$OUTPUT_DIR/$CFG/summary.json"
    if [ -f "$SUMMARY" ]; then
        echo ""
        echo "--- $CFG ---"
        cat "$SUMMARY"
    else
        echo "--- $CFG --- (no summary found)"
    fi
done
