#!/bin/bash
# =============================================================================
# Complete Racing Line Generation Pipeline
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "=========================================="
echo "  Racing Line Generation Pipeline"
echo "=========================================="
echo ""

# Step 1: Generate initial racing line
echo "[1/3] Generating initial racing line from manual waypoints..."
python3 generate_racing_line_manual.py
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to generate initial racing line"
    exit 1
fi
echo ""

# Step 2: QP smoothing
echo "[2/3] Smoothing path with QP optimization..."
python3 qp_smooth_path.py
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to smooth path"
    exit 1
fi
echo ""

# Step 3: Collision check and finalize
echo "[3/3] Checking collisions and generating final path..."
python3 collision_check.py
if [ $? -ne 0 ]; then
    echo "ERROR: Failed collision check"
    exit 1
fi

echo ""
echo "=========================================="
echo "  SUCCESS!"
echo "=========================================="
echo ""
echo "Generated files:"
echo "  - initial_racing_line.yaml"
echo "  - racing_line_preview.png"
echo "  - smoothed_racing_line.yaml"
echo "  - smoothing_comparison.png"
echo "  - final_racing_line.yaml"
echo "  - clearance_visualization.png"
echo ""
echo "Final racing line also copied to:"
echo "  - src/rc_model_description/maps/final_racing_line.yaml"
echo ""
echo "Next steps:"
echo "  1. Review the visualization images"
echo "  2. Update nav2_params_rc.yaml with racing planner config"
echo "  3. Launch Nav2 and test!"
echo ""

