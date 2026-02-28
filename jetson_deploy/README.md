# Jetson Orin Nano 8GB — DA3 + YOLO Deployment

Real-time depth estimation and object detection on the Ackermann robot,
replacing the distilled student model with TensorRT-optimized teacher models.

## Hardware

- **Jetson Orin Nano 8GB** (JetPack 6.x)
- Orbbec Femto Bolt (RGB camera only — monocular depth, no ToF needed)

## Architecture

```
Camera (RGB) ──► DA3-Small (TensorRT FP16) ──► /depth (30FC1)
                                              ──► /depth_colored (viz)
             ──► YOLOv8-nano (TensorRT FP16) ──► /detections
                                              ──► /detection_image (viz)

/depth ──────────────────────► Nav2 Costmap (obstacle layer)
/detections + /depth ────────► Nav2 Costmap (semantic layer)
```

## Memory Budget (8 GB)

| Component        | Memory   | Notes                         |
|------------------|----------|-------------------------------|
| Ubuntu + JetPack | ~2.0 GB  | OS, CUDA runtime              |
| DA3-Small TRT    | ~1.2 GB  | 308x308, FP16                 |
| YOLOv8-nano TRT  | ~0.5 GB  | 640x640, FP16                 |
| ROS2 + Nav2      | ~1.5 GB  | Costmap, planner, controller  |
| Buffers          | ~1.5 GB  | Image topics, TF, point cloud |
| **Total**        | **~6.7 GB** | ~1.3 GB headroom           |

## Expected Performance

| Model       | Resolution | FPS    | Latency |
|-------------|-----------|--------|---------|
| DA3-Small   | 308x308   | 40-50  | ~22 ms  |
| YOLOv8-nano | 640x640   | 60-80  | ~12 ms  |

With staggered execution (DA3 every frame, YOLO every 3rd):
~25-30 Hz effective depth rate.

## Setup

### 1. DA3 (GerdsenAI Wrapper)

```bash
# Clone the DA3 ROS2 wrapper
cd ~/
git clone https://github.com/GerdsenAI/GerdsenAI-Depth-Anything-3-ROS2-Wrapper.git da3_ros2
cd da3_ros2

# First run builds Docker + TensorRT engine (~15-20 min)
./run.sh --camera /dev/video0

# Headless (SSH):
./run.sh --camera /dev/video0 --no-display
```

Published topics:
- `/depth_anything_3/depth` (sensor_msgs/Image, 32FC1) — metric depth in metres
- `/depth_anything_3/depth_colored` (sensor_msgs/Image, BGR8) — visualization

### 2. YOLO

```bash
# Install ultralytics (has built-in Jetson TensorRT export)
pip install ultralytics

# Export YOLOv8-nano to TensorRT (one-time, ~5 min)
yolo export model=yolov8n.pt format=engine device=0 half=True imgsz=640
```

See `yolo_node.py` for the ROS2 wrapper.

### 3. Launch Everything

```bash
# Terminal 1: DA3 depth
cd ~/da3_ros2 && ./run.sh --camera /dev/video0 --no-display

# Terminal 2: YOLO detection
ros2 run jetson_deploy yolo_node

# Terminal 3: Nav2
ros2 launch rc_nav_bridge navigation.launch.py
```

Or use the combined launch file:
```bash
ros2 launch jetson_deploy perception.launch.py
```
