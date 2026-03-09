#!/usr/bin/env bash
#
# LILocBench full benchmark: costmap ablation + localization accuracy.
# Runs all 5 configs, records costmap stats AND estimated poses,
# then computes APE/RPE against ground truth.
#
# Usage:
#   bash run_lilocbench_benchmark.sh [SEQUENCE] [BAG_PATH] [DATA_DIR] [MAP_PG] [GT_FILE]
#
# Example:
#   bash run_lilocbench_benchmark.sh dynamics_0 \
#       ~/MS_Project/lilocbench/bags/dynamics_0_no_cam_ros2 \
#       ~/MS_Project/lilocbench \
#       ~/maps/lilocbench_posegraph \
#       ~/MS_Project/lilocbench/gt_dynamics_0.txt

set -eo pipefail

SEQUENCE="${1:-dynamics_0}"
BAG_PATH="${2:-$HOME/MS_Project/lilocbench/bags/${SEQUENCE}_no_cam_ros2}"
DATA_DIR="${3:-$HOME/MS_Project/lilocbench}"
MAP_PG="${4:-$HOME/maps/lilocbench_posegraph}"
GT_FILE="${5:-$HOME/MS_Project/lilocbench/gt_${SEQUENCE}.txt}"
RATE="${6:-1.0}"
OUTPUT_DIR="$HOME/maps/lilocbench_benchmark/${SEQUENCE}"

source /opt/ros/humble/setup.bash
source "$HOME/nchsb_ws/install/setup.bash"

CONFIGS=("L" "L+S" "L+D" "D" "L+D+dyn")

echo "============================================="
echo "LILOCBENCH FULL BENCHMARK: $SEQUENCE"
echo "  Bag:      $BAG_PATH"
echo "  Data dir: $DATA_DIR"
echo "  Map:      $MAP_PG"
echo "  GT:       $GT_FILE"
echo "  Rate:     ${RATE}x"
echo "  Output:   $OUTPUT_DIR"
echo "  Configs:  ${CONFIGS[*]}"
echo "============================================="

EVAL_SCRIPT="$HOME/MS_Project/NCHSB/rc_hardware_bringup/scripts/eval_localization.py"

for CFG in "${CONFIGS[@]}"; do
    echo ""
    echo ">>> Starting config: $CFG"
    echo "    $(date)"

    CFG_DIR="$OUTPUT_DIR/$CFG"
    mkdir -p "$CFG_DIR"

    RUN_RATE="$RATE"
    if [[ "$CFG" == "L+D" || "$CFG" == "D" || "$CFG" == "L+D+dyn" ]]; then
        RUN_RATE="0.3"
        echo "    (DA3 config — forcing rate=${RUN_RATE}x)"
    fi

    # Launch ablation
    ros2 launch rc_hardware_bringup lilocbench_ablation.launch.py \
        costmap_config:="$CFG" \
        bag_path:="$BAG_PATH" \
        data_dir:="$DATA_DIR" \
        map_posegraph:="$MAP_PG" \
        rate:="$RUN_RATE" &
    LAUNCH_PID=$!

    sleep 12

    # Start costmap evaluator
    ros2 run rc_hardware_bringup costmap_evaluator.py --ros-args \
        -p config_label:="$CFG" \
        -p output_dir:="$OUTPUT_DIR" \
        -p snapshot_interval:=5.0 &
    EVAL_PID=$!

    # Start pose logger
    POSE_FILE="$CFG_DIR/estimated_poses.txt"
    ros2 run rc_hardware_bringup pose_logger.py --ros-args \
        -p output_file:="$POSE_FILE" \
        -p rate:=20.0 &
    POSE_PID=$!

    # Wait for bag to finish
    BAG_DUR=160
    WAIT_SECS=$(python3 -c "import math; print(int(math.ceil($BAG_DUR / $RUN_RATE)) + 30)")
    echo "    Waiting up to ${WAIT_SECS}s for playback..."
    sleep "$WAIT_SECS"

    echo "    Shutting down config: $CFG"
    kill $POSE_PID 2>/dev/null || true
    sleep 1
    kill $EVAL_PID 2>/dev/null || true
    sleep 2
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 5

    # Clean up lingering processes
    pkill -f "lilocbench_replayer" 2>/dev/null || true
    pkill -f "da3_inference" 2>/dev/null || true
    pkill -f "da3_to_pointcloud" 2>/dev/null || true
    pkill -f "costmap_evaluator" 2>/dev/null || true
    pkill -f "pose_logger" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "yolo_node" 2>/dev/null || true
    pkill -f "dynamic_inflation" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "behavior_server" 2>/dev/null || true
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "lifecycle_manager" 2>/dev/null || true
    pkill -f "velocity_smoother" 2>/dev/null || true
    pkill -f "smoother_server" 2>/dev/null || true
    pkill -f "waypoint_follower" 2>/dev/null || true
    sleep 3

    # Compute localization metrics
    if [ -f "$POSE_FILE" ] && [ -f "$GT_FILE" ]; then
        POSE_COUNT=$(wc -l < "$POSE_FILE")
        echo "    Estimated poses: $POSE_COUNT"
        if [ "$POSE_COUNT" -gt 10 ]; then
            python3 "$EVAL_SCRIPT" \
                --gt "$GT_FILE" \
                --est "$POSE_FILE" \
                --output "$CFG_DIR/localization_results.json" \
                --align 2>&1 | sed 's/^/    /'
        else
            echo "    WARNING: Too few poses ($POSE_COUNT), skipping eval"
        fi
    else
        echo "    WARNING: Missing pose file or GT, skipping eval"
    fi

    echo "    Done: $CFG — results in $CFG_DIR/"
done

echo ""
echo "============================================="
echo "BENCHMARK COMPLETE: $SEQUENCE"
echo "Results: $OUTPUT_DIR/"
echo "============================================="

# Print summary
echo ""
echo "=== COSTMAP SUMMARY ==="
for CFG in "${CONFIGS[@]}"; do
    SUMMARY="$OUTPUT_DIR/$CFG/summary.json"
    if [ -f "$SUMMARY" ]; then
        echo "--- $CFG ---"
        python3 -c "import json; d=json.load(open('$SUMMARY')); print(f'  Frames: {d[\"total_frames\"]}, Mean occ: {d[\"occupied_cells_mean\"]:.0f}')"
    fi
done

echo ""
echo "=== LOCALIZATION SUMMARY ==="
for CFG in "${CONFIGS[@]}"; do
    LOC="$OUTPUT_DIR/$CFG/localization_results.json"
    if [ -f "$LOC" ]; then
        echo "--- $CFG ---"
        python3 -c "
import json
d=json.load(open('$LOC'))
print(f'  APE RMSE: {d[\"ape_rmse\"]:.3f}m, Mean: {d[\"ape_mean\"]:.3f}m, Max: {d[\"ape_max\"]:.3f}m')
print(f'  RPE trans: {d[\"rpe_trans_rmse\"]:.3f}m, rot: {d[\"rpe_rot_rmse_deg\"]:.2f}°')
"
    fi
done
