#!/usr/bin/env bash
#
# Run all 4 costmap ablation configs sequentially.
# Each run replays the corridor rosbag, records costmap stats,
# then shuts down before starting the next config.
#
# Usage:
#   bash run_ablation.sh [BAG_PATH] [MAP_POSEGRAPH] [RATE]
#
# Example:
#   bash run_ablation.sh \
#       ~/MS_Project/NCHSB/rosbags/rgbd_imu_20260228_003828_0.mcap \
#       ~/maps/corridor_posegraph \
#       0.5

set -euo pipefail

BAG_PATH="${1:-$HOME/MS_Project/NCHSB/rosbags/rgbd_imu_20260228_003828_0.mcap}"
MAP_PG="${2:-$HOME/maps/corridor_posegraph}"
RATE="${3:-1.0}"
OUTPUT_DIR="$HOME/maps/ablation_results"

source /opt/ros/humble/setup.bash
source "$HOME/nchsb_ws/install/setup.bash"

CONFIGS=("L" "L+S" "L+D" "D")

echo "============================================="
echo "COSTMAP ABLATION STUDY"
echo "  Bag:   $BAG_PATH"
echo "  Map:   $MAP_PG"
echo "  Rate:  ${RATE}x"
echo "  Output: $OUTPUT_DIR"
echo "  Configs: ${CONFIGS[*]}"
echo "============================================="

for CFG in "${CONFIGS[@]}"; do
    echo ""
    echo ">>> Starting config: $CFG"
    echo "    $(date)"

    # DA3 configs are slow; L and L+S can run at full speed
    RUN_RATE="$RATE"
    if [[ "$CFG" == "L+D" || "$CFG" == "D" ]]; then
        RUN_RATE="0.3"
        echo "    (DA3 config — forcing rate=${RUN_RATE}x)"
    fi

    # Launch ablation + evaluator in background
    ros2 launch rc_hardware_bringup costmap_ablation.launch.py \
        costmap_config:="$CFG" \
        bag_path:="$BAG_PATH" \
        map_posegraph:="$MAP_PG" \
        rate:="$RUN_RATE" &
    LAUNCH_PID=$!

    sleep 8

    ros2 run rc_hardware_bringup costmap_evaluator.py --ros-args \
        -p config_label:="$CFG" \
        -p output_dir:="$OUTPUT_DIR" \
        -p snapshot_interval:=5.0 &
    EVAL_PID=$!

    # Wait for the bag to finish (mcap_replayer publishes 'Playback finished')
    # Estimate: bag_duration / rate + startup buffer
    BAG_DUR=82
    WAIT_SECS=$(python3 -c "import math; print(int(math.ceil($BAG_DUR / $RUN_RATE)) + 20)")
    echo "    Waiting up to ${WAIT_SECS}s for playback..."
    sleep "$WAIT_SECS"

    echo "    Shutting down config: $CFG"
    kill $EVAL_PID 2>/dev/null || true
    sleep 2
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 5

    # Clean up any lingering ROS nodes from this run
    pkill -f "mcap_replayer" 2>/dev/null || true
    pkill -f "da3_inference" 2>/dev/null || true
    pkill -f "da3_to_pointcloud" 2>/dev/null || true
    pkill -f "costmap_evaluator" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "static_transform_publisher.*odom.*base_link" 2>/dev/null || true
    sleep 3

    echo "    Done: $CFG — results in $OUTPUT_DIR/$CFG/"
done

echo ""
echo "============================================="
echo "ALL ABLATIONS COMPLETE"
echo "Results: $OUTPUT_DIR/"
echo "============================================="

# Print summary
for CFG in "${CONFIGS[@]}"; do
    SUMMARY="$OUTPUT_DIR/$CFG/summary.json"
    if [ -f "$SUMMARY" ]; then
        echo ""
        echo "--- $CFG ---"
        cat "$SUMMARY"
    fi
done
