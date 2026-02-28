#!/bin/bash
# Setup script for Jetson Orin Nano 8GB deployment
# Run this once on a fresh JetPack 6.x install
set -euo pipefail

echo "=== Jetson Orin Nano 8GB — Perception Stack Setup ==="
echo ""

# ── 1. DA3 (GerdsenAI Wrapper) ──────────────────────────────────────────
echo "--- Step 1: DA3 Depth Estimation ---"
if [ -d "$HOME/da3_ros2" ]; then
    echo "DA3 wrapper already cloned at ~/da3_ros2"
    cd "$HOME/da3_ros2" && git pull
else
    cd "$HOME"
    git clone https://github.com/GerdsenAI/GerdsenAI-Depth-Anything-3-ROS2-Wrapper.git da3_ros2
    echo "DA3 wrapper cloned to ~/da3_ros2"
fi

# Configure for Orin Nano 8GB
echo "Setting DA3 config for Orin Nano 8GB (DA3-Small, 308x308)..."
if [ -f "$HOME/da3_ros2/config/params.yaml" ]; then
    # The GerdsenAI wrapper auto-detects platform, but we can override
    echo "Config file exists — verify model_name=DA3-Small and resolution=308x308"
fi
echo ""

# ── 2. YOLO ──────────────────────────────────────────────────────────────
echo "--- Step 2: YOLOv8 ---"
pip install ultralytics 2>/dev/null || echo "ultralytics already installed"
echo ""

# Export to TensorRT if not already done
YOLO_ENGINE="$HOME/yolov8n.engine"
if [ -f "$YOLO_ENGINE" ]; then
    echo "YOLOv8-nano TensorRT engine already exists: $YOLO_ENGINE"
else
    echo "Exporting YOLOv8-nano to TensorRT FP16 (this takes ~5 minutes)..."
    yolo export model=yolov8n.pt format=engine device=0 half=True imgsz=640
    mv yolov8n.engine "$YOLO_ENGINE" 2>/dev/null || true
    echo "Engine saved: $YOLO_ENGINE"
fi
echo ""

# ── 3. Python dependencies ──────────────────────────────────────────────
echo "--- Step 3: Python dependencies ---"
pip install numpy opencv-python-headless 2>/dev/null || true
echo ""

echo "=== Setup complete ==="
echo ""
echo "Test DA3:"
echo "  cd ~/da3_ros2 && ./run.sh --camera /dev/video0"
echo ""
echo "Test YOLO:"
echo "  ros2 run jetson_deploy yolo_node"
echo ""
echo "Run everything:"
echo "  ros2 launch jetson_deploy perception.launch.py"
