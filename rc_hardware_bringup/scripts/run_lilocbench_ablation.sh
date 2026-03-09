#!/usr/bin/env bash
#
# Run all 5 costmap ablation configs on LILocBench dynamics_0.
# Each run replays the bag + camera images, records costmap stats,
# then shuts down before starting the next config.
#
# Usage:
#   bash run_lilocbench_ablation.sh [BAG_PATH] [DATA_DIR] [MAP_POSEGRAPH] [RATE]
#
# Example:
#   bash run_lilocbench_ablation.sh \
#       ~/MS_Project/lilocbench/bags/dynamics_0_no_cam_ros2 \
#       ~/MS_Project/lilocbench \
#       ~/maps/lilocbench_posegraph \
#       0.3

set -eo pipefail

BAG_PATH="${1:-$HOME/MS_Project/lilocbench/bags/dynamics_0_no_cam_ros2}"
DATA_DIR="${2:-$HOME/MS_Project/lilocbench}"
MAP_PG="${3:-$HOME/maps/lilocbench_posegraph}"
RATE="${4:-1.0}"
OUTPUT_DIR="$HOME/maps/lilocbench_ablation_results"

source /opt/ros/humble/setup.bash
source "$HOME/nchsb_ws/install/setup.bash"

CONFIGS=("L" "L+S" "L+D" "D" "L+D+dyn")

echo "============================================="
echo "LILOCBENCH COSTMAP ABLATION STUDY"
echo "  Bag:      $BAG_PATH"
echo "  Data dir: $DATA_DIR"
echo "  Map:      $MAP_PG"
echo "  Rate:     ${RATE}x"
echo "  Output:   $OUTPUT_DIR"
echo "  Configs:  ${CONFIGS[*]}"
echo "============================================="

for CFG in "${CONFIGS[@]}"; do
    echo ""
    echo ">>> Starting config: $CFG"
    echo "    $(date)"

    RUN_RATE="$RATE"
    if [[ "$CFG" == "L+D" || "$CFG" == "D" || "$CFG" == "L+D+dyn" ]]; then
        RUN_RATE="0.3"
        echo "    (DA3 config — forcing rate=${RUN_RATE}x)"
    fi

    ros2 launch rc_hardware_bringup lilocbench_ablation.launch.py \
        costmap_config:="$CFG" \
        bag_path:="$BAG_PATH" \
        data_dir:="$DATA_DIR" \
        map_posegraph:="$MAP_PG" \
        rate:="$RUN_RATE" &
    LAUNCH_PID=$!

    sleep 10

    ros2 run rc_hardware_bringup costmap_evaluator.py --ros-args \
        -p config_label:="$CFG" \
        -p output_dir:="$OUTPUT_DIR" \
        -p snapshot_interval:=5.0 &
    EVAL_PID=$!

    # dynamics_0 is ~160s; wait bag_duration / rate + startup buffer
    BAG_DUR=160
    WAIT_SECS=$(python3 -c "import math; print(int(math.ceil($BAG_DUR / $RUN_RATE)) + 30)")
    echo "    Waiting up to ${WAIT_SECS}s for playback..."
    sleep "$WAIT_SECS"

    echo "    Shutting down config: $CFG"
    kill $EVAL_PID 2>/dev/null || true
    sleep 2
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 5

    pkill -f "lilocbench_replayer" 2>/dev/null || true
    pkill -f "da3_inference" 2>/dev/null || true
    pkill -f "da3_to_pointcloud" 2>/dev/null || true
    pkill -f "costmap_evaluator" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "yolo_node" 2>/dev/null || true
    pkill -f "dynamic_inflation" 2>/dev/null || true
    sleep 3

    echo "    Done: $CFG — results in $OUTPUT_DIR/$CFG/"
done

echo ""
echo "============================================="
echo "ALL LILOCBENCH ABLATIONS COMPLETE"
echo "Results: $OUTPUT_DIR/"
echo "============================================="

for CFG in "${CONFIGS[@]}"; do
    SUMMARY="$OUTPUT_DIR/$CFG/summary.json"
    if [ -f "$SUMMARY" ]; then
        echo ""
        echo "--- $CFG ---"
        cat "$SUMMARY"
    fi
done
