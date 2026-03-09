# ROS2 DA3 Costmap — Command Log

## Environment Setup (run once per terminal)
**IMPORTANT**: Do NOT activate conda for ROS2 nodes. ROS2 Humble needs system Python 3.10.
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash
# DO NOT run: conda activate nchsb_ml (breaks rclpy)
```

## Workspace Build
```bash
# One-time: install build deps in conda env
pip install catkin_pkg empy lark

# Build (from workspace root)
cd ~/nchsb_ws
colcon build --symlink-install
```

---

## Session 1 — 2026-03-01: Initial rosbag test with sensor depth

### Rosbag Info
- **File**: `~/MS_Project/NCHSB/rosbags/rgbd_imu_20260228_003828_0.mcap`
- **Duration**: ~81 seconds, 26909 messages
- **Topics**:
  | Topic | Type | Count |
  |-------|------|-------|
  | `/camera/color/image_raw` | sensor_msgs/Image | 2454 |
  | `/camera/depth/image_raw` | sensor_msgs/Image | 2455 |
  | `/camera/color/camera_info` | sensor_msgs/CameraInfo | 2454 |
  | `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 2454 |
  | `/scan` | sensor_msgs/LaserScan | 815 |
  | `/camera/gyro_accel/sample` | sensor_msgs/Imu | 16276 |
  | `/tf_static` | tf2_msgs/TFMessage | 1 |
- **Note**: Recorded with ROS2 Jazzy. `ros2 bag play` fails on Humble due to Jazzy metadata format (`history: unknown`). Use `mcap_replayer.py` instead.
- No DA3 depth topic in bag; using sensor depth `/camera/depth/image_raw` for pipeline validation.

### Step 1: Replay rosbag (Terminal 1)
**IMPORTANT**: Do NOT activate conda. ROS2 nodes need system Python 3.10.
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash

# Custom replayer (handles Jazzy->Humble metadata incompatibility)
python3 ~/MS_Project/NCHSB/rc_hardware_bringup/rc_hardware_bringup/mcap_replayer.py \
  --ros-args \
  -p bag_path:=$HOME/MS_Project/NCHSB/rosbags/rgbd_imu_20260228_003828_0.mcap \
  -p rate:=1.0 \
  -p loop:=true
```

### Step 2: DA3-to-PointCloud node (Terminal 2)
Using sensor depth (`/camera/depth/image_raw`) instead of DA3 depth for pipeline validation.
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash

# Full visualization (relaxed height filter):
python3 ~/MS_Project/NCHSB/rc_hardware_bringup/rc_hardware_bringup/da3_to_pointcloud.py \
  --ros-args \
  -p depth_topic:=/camera/depth/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p min_height:=-2.0 \
  -p max_height:=2.0 \
  -p camera_height:=0.9 \
  -p downsample:=2

# For Nav2 costmap (obstacle-band only, default height filter):
# python3 ~/MS_Project/NCHSB/rc_hardware_bringup/rc_hardware_bringup/da3_to_pointcloud.py \
#   --ros-args \
#   -p depth_topic:=/camera/depth/image_raw \
#   -p camera_info_topic:=/camera/color/camera_info
```

### Step 3: Verify PointCloud2 output (Terminal 3)
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash

# Check message rate (expected: ~28-30 Hz)
ros2 topic hz /da3/pointcloud
```

### Step 4: Static TF for floor alignment (Terminal 4)
Publishes `world -> camera_link`. The bag's internal `/tf_static` handles the rest
of the chain: `camera_link -> camera_depth_frame -> camera_color_frame -> camera_color_optical_frame`.
Pitch derived from IMU accelerometer gravity vector (~5-7° downward, handheld).
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.05 \
  --qx 0.000000 --qy -0.061000 --qz 0.000000 --qw 0.998100 \
  --frame-id world --child-frame-id camera_link
```
**Tuning tips**:
- `--z`: Camera height above grid. Best value found: 0.05 (bag was handheld at ~waist level).
- `--qy`: Controls pitch. More negative = more downward tilt correction. Range tried: -0.034 to -0.061.
- A static TF is an approximation. Handheld recordings have continuous tilt variation; slight residual tilt (~2-3°) is expected and acceptable.

### Step 5: Visualize in RViz2 (Terminal 5)
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash
rviz2
```
RViz setup:
1. Set **Fixed Frame** to `world` (NOT `camera_color_optical_frame`)
2. **Add** -> By topic -> `/da3/pointcloud` -> PointCloud2
3. PointCloud2 settings: Size=0.01, Color Transformer=AxisColor, Axis=Z
4. DO NOT add Image display in RViz (causes freeze); use rqt_image_view instead

### Step 6: View camera feed (Terminal 6)
```bash
source /opt/ros/humble/setup.bash
source ~/nchsb_ws/install/setup.bash
rqt_image_view /camera/color/image_raw
```

### Results (2026-03-01)
- **Replayer**: 8 topics, 26909 msgs over 81.4s (real-time), loops seamlessly with wall-clock restamping
- **da3_to_pointcloud**: Camera intrinsics received (fx=747.8, fy=747.5)
- **PointCloud2 rate**: ~28 Hz (downsample=4), ~8 Hz (downsample=2, denser cloud)
- **IMU gravity analysis**: Camera tilted 3.9° downward, 0.5° roll (handheld corridor recording)
- **TF chain**: `world -> camera_link -> camera_depth_frame -> camera_color_frame -> camera_color_optical_frame` (bag provides all except world->camera_link)
- **RViz visualization**: Corridor geometry (floor, walls, ceiling) visible in `world` frame. Floor approximately aligned with grid plane. Residual ~2-3° tilt from handheld motion is acceptable.
- **Pipeline validated end-to-end** with sensor depth as DA3 stand-in

### Key findings
1. `ros2 bag play` cannot replay Jazzy-recorded MCAP bags on Humble (metadata format mismatch). Custom `mcap_replayer.py` solves this.
2. ROS2 nodes MUST use system Python 3.10. Conda env (Python 3.11) breaks `rclpy` C extensions.
3. RViz Image display at 30 Hz freezes PointCloud2; use `rqt_image_view` separately.
4. Bag timestamps must be remapped to wall clock for seamless looping; original bag timestamps cause RViz TF lookup failures on loop restart.
5. Static TF should target `camera_link` (root of bag's TF tree), not individual optical frames.

---

## Errors & Fixes Log

| Date | Error | Fix |
|------|-------|-----|
| 2026-03-01 | `No module named 'catkin_pkg'` during `colcon build` | `pip install catkin_pkg empy lark` in conda env |
| 2026-03-01 | `yaml-cpp: error at line 1, column 12: bad conversion` with `ros2 bag play` | Jazzy metadata incompatible with Humble; created `mcap_replayer.py` using `mcap` Python lib |
| 2026-03-01 | `No module named 'rclpy._rclpy_pybind11'` (Python 3.11) | Don't activate conda for ROS2 nodes; use system Python 3.10 |
| 2026-03-01 | `'dict' object has no attribute 'append'` in replayer | Renamed `self._publishers` to `self._topic_pubs` to avoid shadowing rclpy internal |
| 2026-03-01 | `TypeError: Expected LaserScan, got mcap_ros2._dynamic.LaserScan` | Use `rclpy.serialization.deserialize_message()` with raw bytes instead of `iter_decoded_messages()` |
| 2026-03-01 | `publisher's context is invalid` in timer callback | Moved replay loop to daemon thread; timer callback was blocking `rclpy.spin()` |
| 2026-03-01 | da3_to_pointcloud QoS mismatch (BEST_EFFORT vs RELIABLE) | Changed node QoS to RELIABLE to match rosbag |
| 2026-03-01 | RViz Image display freezes PointCloud2 rendering | Use `rqt_image_view` in separate terminal instead |
| 2026-03-01 | Point cloud misaligned with RViz grid (floor not flat) | Computed static TF from IMU gravity vector; set Fixed Frame to `world` |
| 2026-03-01 | `rqt_image_view: command not found` | Must `source /opt/ros/humble/setup.bash` first; or use `ros2 run rqt_image_view rqt_image_view` |
| 2026-03-01 | PointCloud2 stops after replayer loop (timestamps jump back) | Restamp all messages with wall-clock time in `mcap_replayer.py` |
| 2026-03-01 | `world` fixed frame: floor below grid; `camera_color_frame` works better | Publish `world -> camera_link` (not `camera_color_optical_frame`); bag's `/tf_static` handles `camera_link -> ... -> camera_color_optical_frame` |
| 2026-03-01 | RViz "discarding message because the queue is full" | Normal at high rate with dense cloud; not an error |

---

## Next Steps
1. **DA3 inference ROS2 node**: Subscribe to `/camera/color/image_raw`, run DA3-Small, publish to `/depth_anything_3/depth`. This replaces sensor depth with monocular depth predictions — the core of the project.
2. **Nav2 costmap integration**: Launch Nav2 with `nav2_da3_costmap.yaml` to build actual costmaps from the PointCloud2.
3. **Dynamic inflation testing**: Launch `dynamic_inflation.py` with different strategies to compare costmap behavior.

---

## Notes
- **Python version**: ROS2 Humble uses system Python 3.10. The `nchsb_ml` conda env uses Python 3.11. DO NOT activate conda for ROS2 nodes. Only use conda for standalone ML scripts.
- **Rosbag metadata backup**: Original Jazzy metadata saved at `rosbags/metadata.yaml.jazzy_backup`.
- For real DA3 depth testing, either run DA3 inference as a separate node or record a new bag with DA3 depth included.
- `--symlink-install` means edits to Python source files in `NCHSB/rc_hardware_bringup/rc_hardware_bringup/` take effect immediately without rebuilding.
