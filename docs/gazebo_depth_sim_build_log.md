# Gazebo Depth-Error Simulation -- Build Log

**Project:** TIER 2 closed-loop Gazebo simulation study for RA-L paper  
**Branch:** `feat/gazebo-depth-sim` (both NCHSB and ml_pipeline)  
**Started:** 2026-02-27  
**Plan file:** `~/.cursor/plans/gazebo_closed-loop_sim_6fcc38fb.plan.md`

---

## Log Format

Each entry records:
- **Phase / Task** -- which plan phase this belongs to
- **What was done** -- action taken, files created/modified
- **Commands run** -- exact ROS/build/git commands
- **Output / Result** -- what happened, key numbers
- **Errors & Fixes** -- any issues and how they were resolved

---

## Phase 0: Branch Setup

**Date:** 2026-02-27  
**Goal:** Create isolated `feat/gazebo-depth-sim` branches on both repos so Tier 2 work does not contaminate existing branches.

### Step 0.1: Pre-branch cleanup -- NCHSB

Both repos had uncommitted changes on their working branches. Committed them first to ensure clean branch points.

**NCHSB** (`feat/corridor-social-nav`):
- 148 files staged (modified configs, new SLAM screenshots, ablation launch files, replayer nodes, build log updates)
- Added `rosbags/` to `.gitignore` (8.1GB directory, too large for git)
- `.gitignore` already excluded `build/`, `install/`, `log/`, `*.db3`, `*.bag`, `aws-robomaker-hospital-world/`

**Command:**
```bash
cd ~/MS_Project/NCHSB
# Added rosbags/ to .gitignore
git add -A
git commit -m "Pre-branch snapshot: SLAM screenshots, ablation configs, replayers, and build log updates"
git push origin feat/corridor-social-nav
```

**Result:** Commit `9f46b8f`, pushed successfully.

### Step 0.2: Pre-branch cleanup -- ml_pipeline

**ml_pipeline** (`v3-da3-efficientvit`):
- 1 file modified: `changelog.tex` (+251 lines -- V6/V6.2/V8/V9 corridor evals, Vivek's HPC training runs)

**Command:**
```bash
cd ~/MS_Project/ml_pipeline
git add changelog.tex
git commit -m "Update changelog: V6/V6.2/V8/V9 corridor evals, Vivek's HPC training runs"
git push origin v3-da3-efficientvit
```

**Result:** Commit `6af35d0`, pushed successfully.

### Step 0.3: Create feature branches

**Commands:**
```bash
# NCHSB
cd ~/MS_Project/NCHSB
git checkout -b feat/gazebo-depth-sim
git push -u origin feat/gazebo-depth-sim

# ml_pipeline
cd ~/MS_Project/ml_pipeline
git checkout -b feat/gazebo-depth-sim
git push -u origin feat/gazebo-depth-sim
```

**Result:**
- NCHSB: Branch `feat/gazebo-depth-sim` created from `9f46b8f` (parent: `feat/corridor-social-nav`), pushed to `origin`
- ml_pipeline: Branch `feat/gazebo-depth-sim` created from `6af35d0` (parent: `v3-da3-efficientvit`), pushed to `origin`

**Verification:**
```
NCHSB:      9f46b8f on feat/gazebo-depth-sim, tracking origin/feat/gazebo-depth-sim
ml_pipeline: 6af35d0 on feat/gazebo-depth-sim, tracking origin/feat/gazebo-depth-sim
```

### Step 0.4: Build log created

**File:** `NCHSB/docs/gazebo_depth_sim_build_log.md` (this file)

Tracks every action, command, file, output, and error for the entire Gazebo depth-error simulation study.

### Errors & Fixes

None. Clean branch creation.

---

## Phase 0 Status: COMPLETE

**Summary:**
- Both repos on `feat/gazebo-depth-sim` branch
- Parent branches committed and pushed (no dirty state left behind)
- Build log initialized
- Ready to proceed to Phase 1: Add RGBD Camera to Robot

---

## Phase 1: Add RGBD Camera to Robot

**Date:** 2026-02-27  
**Goal:** Add an RGBD camera sensor to the simulated robot matching the real Orbbec Femto Bolt specs (640x480, 15Hz, 0.1-5.0m). Bridge camera topics to ROS2.

### Design Decisions

- **Camera mount position:** Copied exactly from `rc_hardware_bringup/urdf/rc_model_hardware.urdf.xacro` -- `xyz="0.0 0.05 0.08"` relative to `chassis_link`, rotated `rpy="0 0 -1.5707"` (90 deg yaw to face forward since chassis_link's Y-axis is rotated from base_link)
- **Optical frames:** Both `camera_color_optical_frame` and `camera_depth_optical_frame` with the standard `-90 deg roll, -90 deg yaw` rotation to get Z-forward camera convention
- **Sensor type:** `rgbd_camera` (Gazebo Ignition Fortress native sensor), not `camera` + separate `depth_camera`
- **Update rate:** 15 Hz (not 30 Hz) -- balances between the real sensor's 30 Hz and sim performance on GTX 1060
- **Resolution:** 640x480 matching real Femto Bolt
- **Depth range:** 0.1m - 10.0m (real sensor 0.1m - 5.0m, extended for sim flexibility)
- **Bridge remappings:** Gazebo publishes `/camera/image`, `/camera/depth_image`, `/camera/camera_info`; remapped to `/camera/color/image_raw`, `/camera/depth`, `/camera/color/camera_info` to match real robot's topic names

### Files Modified

**1. `src/rc_model_description/urdf/links.xacro`**  
Added `camera_link` (small black box visual), `camera_color_optical_frame`, `camera_depth_optical_frame` links. Attached `<gazebo reference="camera_link">` block with the `rgbd_camera` sensor (topic=`camera`, 640x480, 15Hz, hfov=60deg, clip 0.1-10m). Uses `gz_frame_id=camera_color_optical_frame` so published images have the correct TF frame.

**2. `src/rc_model_description/urdf/joints.xacro`**  
Added three fixed joints: `camera_joint` (chassis→camera_link), `camera_depth_optical_joint` and `camera_color_optical_joint` (camera_link→optical frames). Mount position and optical frame rotations match the real hardware URDF exactly.

**3. `src/rc_model_description/launch/fortress_bringup.launch.py`**  
Added `camera_bridge` Node using `ros_gz_bridge` with three topic bridges:
- `/camera/image` → `/camera/color/image_raw` (RGB, gz.msgs.Image → sensor_msgs/Image)
- `/camera/depth_image` → `/camera/depth` (Depth, gz.msgs.Image → sensor_msgs/Image)
- `/camera/camera_info` → `/camera/color/camera_info` (CameraInfo)

Added `camera_bridge` to the LaunchDescription return list, between `scan_bridge` and `imu_bridge`.

### Errors & Fixes

None during implementation. Code follows existing patterns (LiDAR sensor block and bridge).

### Checkpoint 1 (Manual Test -- YOU RUN):

**Step 1: Build**
```bash
cd ~/MS_Project/NCHSB
colcon build --packages-select rc_model_description
source install/setup.bash
```

**Step 2: Launch Gazebo + Robot**
```bash
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf spawn_x:=-6.5 spawn_y:=0.0 spawn_yaw:=0.0
```

**Step 3: Verify topics (new terminal)**
```bash
source ~/MS_Project/NCHSB/install/setup.bash
ros2 topic list | grep -E "camera|scan"
```
Expected output:
```
/camera/color/camera_info
/camera/color/image_raw
/camera/depth
/scan
/scan/points
```

**Step 4: Check camera publish rate**
```bash
ros2 topic hz /camera/depth
```
Expected: ~15 Hz

**Step 5: Check LiDAR still works**
```bash
ros2 topic hz /scan
```
Expected: ~25 Hz

**Step 6: Visualize in RViz2**
RViz2 should auto-launch. Add two Image displays:
- Topic: `/camera/color/image_raw` -- should show corridor RGB view
- Topic: `/camera/depth` -- should show depth map (darker = closer)

**Pass/Fail Criteria:**
- [ ] All 5 camera+scan topics appear in `ros2 topic list`
- [ ] `/camera/depth` publishes at ~10 Hz
- [ ] `/scan` still publishes at ~25 Hz
- [ ] RGB image shows the corridor in RViz2
- [ ] Depth image shows corridor geometry in RViz2
- [ ] Robot spawns and is visible in Gazebo GUI

### Error: RGBD Camera Blocks Gazebo Initialization (2026-03-09)

**Symptom:** After adding the RGBD camera, launching Gazebo caused:
1. Entity creation timed out (`Request to create entity from service [/world/default/create] timed out..`)
2. gz_ros2_control never initialized (no controller_manager output)
3. Both controller spawners failed: `Could not contact service /controller_manager/list_controllers`
4. Robot spawned but had no steering/drive (controllers dead)

**Root Cause:** Dual-GPU system (Intel UHD 630 + NVIDIA GTX 1060 Mobile). Gazebo's ogre2 renderer was trying to create the RGBD camera's EGL rendering context on the **Intel iGPU**, which fails with `libEGL warning: egl: failed to create dri2 screen`. The gpu_lidar was less demanding and worked anyway, but the full RGBD camera blocked the initialization pipeline, preventing gz_ros2_control from loading.

**Evidence:**
- `/dev/dri/renderD128` = Intel UHD 630 (pci-0000:00:02.0)
- `/dev/dri/renderD129` = NVIDIA GTX 1060 (pci-0000:01:00.0)
- `libEGL warning: egl: failed to create dri2 screen` in old logs (x4)
- Without camera: controller_manager loaded in ~3s after entity creation
- With camera: controller_manager never loaded (30s timeout exceeded)

**Fixes Applied:**

1. **Force NVIDIA GPU rendering** (fortress_bringup.launch.py):
   - Added `__NV_PRIME_RENDER_OFFLOAD=1` environment variable
   - Added `__GLX_VENDOR_LIBRARY_NAME=nvidia` environment variable
   - Added `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`

2. **Reduced camera rendering demands** (links.xacro):
   - Resolution: 640x480 → 320x240 (4x fewer pixels to render)
   - Update rate: 15 Hz → 10 Hz
   - `<visualize>false</visualize>` (removes visualization overlay)

3. **Increased controller spawner timeouts** (fortress_bringup.launch.py):
   - Spawn delay: 2s → 5s (give Gazebo time to load rendering)
   - JSB spawner delay: 8s → 25s (wait for gz_ros2_control)
   - Ackermann spawner delay: 10s → 30s
   - Controller manager timeout: 30s → 120s

**Fix attempt 1 (FAILED):** NVIDIA PRIME env vars + reduced camera (320x240@10Hz) + increased timeouts did NOT fix the issue. The libEGL warnings disappeared (NVIDIA EGL is being used) but the rendering pipeline still hangs during camera sensor initialization, blocking gz_ros2_control from ever loading. The Gazebo GUI shows a black screen with "GUI is not responding."

**Fix attempt 2 (server-only mode):** The root cause is that the camera sensor's rendering initialization hangs the Gazebo process when the GUI is active. The fix is to run Gazebo in **server-only mode** (`-s` flag), which separates physics+sensor processing from the GUI rendering.

Changes to `fortress_bringup.launch.py`:
- Added `gui` launch argument (default: `false`)
- When `gui:=false`: launches with `ign gazebo -s -r` (server-only, no GUI window)
- When `gui:=true`: launches with `ign gazebo -r` (with GUI, may hang on dual-GPU)
- To see the sim visually: open `ign gazebo -g` in a separate terminal after launch
- RViz2 still launches for ROS topic visualization regardless of gui mode

**Launch command (server-only, default):**
```bash
source ~/MS_Project/NCHSB/install/setup.bash
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf spawn_x:=-6.5 spawn_y:=0.0 spawn_yaw:=0.0
```

**To also see Gazebo GUI (separate terminal):**
```bash
ign gazebo -g
```

### Fix attempt 3: Split rgbd_camera into separate camera + depth_camera (2026-03-09)

**Root cause confirmed:** The `rgbd_camera` sensor type creates a combined rendering pipeline that hangs Gazebo Fortress on dual-GPU systems during initialization. Even in server-only mode, the combined sensor blocks `gz_ros2_control`. The `rgbd_camera` type also has [known issues](https://github.com/gazebosim/gz-sensors/issues/239) with frame_id handling in Fortress.

**Solution:** Following the pattern from [acceleration-robotics/ros2-igt IGT One model](https://raw.githubusercontent.com/acceleration-robotics/ros2-igt/e70d16ff4486b0a136afc1f7a1ed6f60fb6340e3/igt_ignition/models/igt_one/model.sdf), split into two separate sensors:
- `camera_color` (type: `camera`) -- publishes `/camera_color/image`, `/camera_color/camera_info`
- `camera_depth` (type: `depth_camera`) -- publishes `/camera_depth/depth_image`, `/camera_depth/camera_info`

Both at 320x240, 10Hz, `visualize=false`. Separate sensors have independent rendering init paths which avoids the deadlock.

**Files changed:**
- `links.xacro`: Replaced single `<sensor type="rgbd_camera">` with two sensors: `<sensor type="camera">` + `<sensor type="depth_camera">`
- `fortress_bringup.launch.py`: Split `camera_bridge` into `camera_color_bridge` and `camera_depth_bridge`, with remappings to match real robot topic names:
  - `/camera_color/image` → `/camera/color/image_raw`
  - `/camera_depth/depth_image` → `/camera/depth`
  - `/camera_color/camera_info` → `/camera/color/camera_info`

**Also kept from fix 2:** Server-only mode (`gui:=false` default), NVIDIA PRIME env vars, increased controller spawner timeouts (120s) and delays (25s/30s).

**Result: FAILED.** `gazebo_sim_logs3.txt` shows same pattern: entity created OK, but `gz_ros2_control` never loads, controller spawners timeout at 120s. Separate `camera` + `depth_camera` sensors still deadlock the ogre2 rendering pipeline on dual-GPU (Intel iGPU + NVIDIA dGPU).

### Fix attempt 4: Xacro camera toggle + software rendering (2026-03-09)

**Root cause update:** The problem is NOT the sensor type (`rgbd_camera` vs separate `camera`/`depth_camera`). ANY camera-type sensor triggers an ogre2 rendering initialization deadlock on this dual-GPU system. The `gpu_lidar` sensor works because it uses a simpler ray-casting pipeline, but camera sensors require full framebuffer rasterization through EGL, which deadlocks regardless of whether NVIDIA PRIME or Intel Mesa is used for hardware rendering.

**Solution: Two-pronged approach:**

1. **Xacro `enable_cameras` toggle** (default `false`):
   - Camera links remain in URDF always (TF tree integrity)
   - Camera *sensors* are conditionally included via `<xacro:if value="$(arg enable_cameras)">`
   - With `enable_cameras:=false` (default), simulation works identically to pre-camera URDF
   - For depth error injection, we'll publish synthetic depth from a ROS node instead of relying on Gazebo camera rendering

2. **Software rendering** (`LIBGL_ALWAYS_SOFTWARE=1`):
   - Removed all three NVIDIA PRIME env vars (`__NV_PRIME_RENDER_OFFLOAD`, `__GLX_VENDOR_LIBRARY_NAME`, `__EGL_VENDOR_LIBRARY_FILENAMES`)
   - Added `LIBGL_ALWAYS_SOFTWARE=1` which forces Mesa's llvmpipe (CPU-based software rasterizer)
   - This bypasses ALL GPU driver issues since rendering happens purely on CPU
   - At 320x240@10Hz, CPU load should be manageable
   - When user wants to test cameras: `enable_cameras:=true`

**Files changed:**
- `links.xacro`: Added `<xacro:arg name="enable_cameras" default="false"/>` and wrapped camera sensor `<gazebo>` block in `<xacro:if>`
- `fortress_bringup.launch.py`:
  - Added `enable_cameras` launch argument (default `false`), passed to xacro via mappings
  - Replaced 3 NVIDIA PRIME `SetEnvironmentVariable` with single `LIBGL_ALWAYS_SOFTWARE=1`
  - Updated `LaunchDescription` return list

**Test plan:**
- **Test A** (`enable_cameras:=false`): Must work -- no camera sensors in URDF, same as original working setup
- **Test B** (`enable_cameras:=true`): Tests software rendering with cameras -- may work with CPU-based ogre2

**Result: FAILED (logs4 + logs5).** Both launches failed -- gz_ros2_control never loaded. Root cause: `LIBGL_ALWAYS_SOFTWARE=1` was set **globally**, which broke the `gpu_lidar`'s ogre2 rendering too (it worked fine with hardware rendering). The env var poisoned ALL rendering, not just camera rendering. Log4 (cameras OFF) confirmed cameras are NOT the issue -- the software rendering broke gpu_lidar.

### Fix attempt 5: Conditional software rendering + restore gui=true (2026-03-09)

**Root cause:** `LIBGL_ALWAYS_SOFTWARE=1` set globally broke `gpu_lidar` rendering. The lidar previously worked fine with hardware GPU (Intel/NVIDIA). Software rendering via llvmpipe is incompatible with ogre2's gpu_lidar implementation on this system. Also, `gui:=false` (server-only mode) meant no Gazebo window appeared, which was confusing.

**Fix:**
1. Made `LIBGL_ALWAYS_SOFTWARE=1` conditional -- only set when `enable_cameras:=true` (uses `IfCondition`)
2. Restored `gui` default to `true` (Gazebo window appears normally)
3. When `enable_cameras:=false` (default), NO rendering env vars are set -- system uses defaults, same as the original working setup

**Files changed:**
- `fortress_bringup.launch.py`: Added `condition=IfCondition(LaunchConfiguration('enable_cameras'))` to `SetEnvironmentVariable`, changed `gui` default from `'false'` to `'true'`

**Result: Partial.** Server-only mode (`gui:=false`) worked -- gz_ros2_control loaded, `/controller_manager/list_controllers` appeared. But GUI mode still showed black screen.

### Root cause found: Leaked GPU rendering contexts (2026-03-09)

After 6+ crashed/killed Gazebo Ignition sessions during debugging, the NVIDIA GPU had leaked rendering contexts that prevented ANY new ogre2 rendering from initializing -- even `ign gazebo shapes.sdf` showed a black screen, and the original `feat/corridor-social-nav` branch also failed.

**Diagnosis steps that confirmed this:**
1. `ign gazebo -v 4 shapes.sdf` → GUI warning: "Waited for 10s for a subscriber to /gazebo/starting_world and got none" (server thread couldn't init rendering)
2. `ign gazebo -v 4 -s shapes.sdf` → Server works perfectly (physics, all services created)
3. `gazebo` (Classic v11.10.2) → GUI renders fine (uses ogre 1.x, different rendering path)
4. `glxinfo | grep "OpenGL renderer"` → NVIDIA GTX 1060 (GPU driver healthy)

**Fix: Reboot.** After `sudo reboot`, `ign gazebo shapes.sdf` rendered correctly. All subsequent launches worked.

**Lesson learned:** Always reboot after multiple crashed Gazebo Ignition sessions on dual-GPU systems. Leaked GPU contexts are not released by killing processes -- they persist until reboot.

### Checkpoint 1: PASSED (2026-03-09)

After reboot, launched with `gui:=true`, `enable_cameras:=false`:
```bash
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf spawn_x:=-6.5 spawn_y:=0.0 spawn_yaw:=0.0 gui:=true
```

**Results:**
- Gazebo GUI opens with corridor world visible
- Robot spawned at (-6.5, 0, 0.12)
- gz_ros2_control loaded successfully
- All 16 controller_manager services available
- joint_state_broadcaster + ackermann_steering_controller active

### Camera sensors: Bridge topic name mismatch (2026-03-09)

**Problem:** After enabling cameras (`enable_cameras:=true`), the RGB and depth topics were not publishing. Controllers loaded fine (gz_ros2_control not blocked), but `/camera/color/image_raw` and `/camera/depth` had no data.

**Diagnosis:**
```bash
ign topic -l | grep camera
# Output:
# /camera_color          ← actual topic (NOT /camera_color/image)
# /camera_depth          ← actual topic (NOT /camera_depth/depth_image)
# /camera_depth/points
# /camera_info

ign topic -e -t /camera_color --num 1   # → prints data (working!)
ign topic -e -t /camera_depth --num 1   # → prints data (working!)
```

**Root cause:** Gazebo Ignition Fortress publishes sensor data on the base `<topic>` name directly, NOT with `/image` or `/depth_image` suffixes. Our bridge was listening to non-existent topics.

| Sensor | Actual Gazebo Topic | Bridge Was Listening To |
|--------|-------------------|----------------------|
| RGB | `/camera_color` | `/camera_color/image` |
| Depth | `/camera_depth` | `/camera_depth/depth_image` |
| Camera Info | `/camera_info` | `/camera_color/camera_info` |

**Fix:** Updated `fortress_bringup.launch.py` bridge arguments:
```python
# camera_color_bridge: /camera_color → /camera/color/image_raw
# camera_depth_bridge: /camera_depth → /camera/depth
# camera_info: /camera_info → /camera/color/camera_info
```

**Result:** Both topics now publish:
- `/camera/color/image_raw`: ~2Hz (320x240 RGB, encoding: `rgb8`)
- `/camera/depth`: ~3Hz (320x240 depth, encoding: `32FC1`)

**Note on LIBGL_ALWAYS_SOFTWARE:** The `IfCondition` sets this when `enable_cameras:=true`, but the NVIDIA driver **rejects it**: `libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device.` The cameras use hardware rendering regardless. The env var is harmless but ineffective on this system.

**Commit:** `38c50e2` "Fix: Correct camera bridge topic names for Fortress"

### RViz TF tree fix (2026-03-09)

**Problem:** RViz showed "Global Status: Error" because the fixed frame was set to `map` but no `map` frame existed in the TF tree. The TF chain was broken at `map → odom`.

**Reference:** This exact issue is documented in `NCHSB/build_logs.tex` (lines 2030-2106) from the original corridor-social-nav development. The expected TF chain is:
```
map → odom → base_link → chassis_link → lidar_link
 |      |          |
AMCL   EKF    robot_state_publisher (static)
```

Without Nav2/AMCL running, the `map → odom` link is missing.

**Fix:** Added a `static_transform_publisher` node to `fortress_bringup.launch.py`:
```python
map_odom_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_odom_static_tf',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    parameters=[{'use_sim_time': True}],
)
```

This publishes an identity transform `map = odom`. In simulation, the odom frame aligns with the world origin, so this is correct. Will be replaced with AMCL in Phase 3 for proper localization.

Also restored RViz fixed frame to `map` and added RGB Camera + Depth Camera Image displays to `urdf.rviz`.

**Result:** RViz Global Status Error resolved. Full TF chain working:
```
map → odom → base_link → chassis_link → lidar_link
                                      → camera_link → camera_color_optical_frame
                                                    → camera_depth_optical_frame
```

**Commit:** `a23eaaa` "Fix: Add static map->odom TF, restore RViz fixed frame to map"

### Depth camera: RViz shows black (visualization issue only) (2026-03-09)

**Problem:** RGB Camera panel in RViz shows the corridor correctly. Depth Camera panel shows solid black.

**Diagnosis:**
```bash
python3 -c "... depth pixel check ..."
# Output:
# Size: 320x240 = 76800 pixels
# Valid (0-100m): 48413
# NaN: 0, Inf: 28387, Zero: 0
# Range: 0.320m - 8.496m, Mean: 1.395m
```

**Root cause:** The depth sensor publishes `32FC1` (float32 single-channel) images. 63% of pixels have valid depth (0.32-8.5m), 37% are `inf` (beyond far clip). RViz's Image display cannot normalize a range that includes `inf` values -- the normalization computes `(value - min) / (max - min)` where `max = inf`, so all finite values map to 0 (black).

**Workaround:** Use `rqt_image_view` which handles float depth images correctly:
```bash
ros2 run rqt_image_view rqt_image_view /camera/depth
```
This shows the depth image properly with auto-normalization that ignores `inf` values.

**Impact on project:** None. The depth error injection experiment reads float32 values directly in Python, not through RViz. The raw depth data (0.32-8.5m, 63% valid) is exactly what we need. The RViz black panel is cosmetic only.

---

## Phase 1 Status: COMPLETE

### Final System Configuration

| Parameter | Value |
|-----------|-------|
| Gazebo Version | Ignition Fortress v6.16.0 |
| GPU | NVIDIA GTX 1060 (driver 580.126.09) |
| Robot Spawn | `spawn_x:=-6.5 spawn_y:=0.0 spawn_z:=0.12` |
| World | `corridor_narrow.sdf` |
| Camera Resolution | 320x240 |
| Camera Update Rate | 10Hz configured, ~2-3Hz actual |
| `enable_cameras` | `false` (default), `true` for camera data |
| `gui` | `true` (default), `false` for headless |

### Sensor Status

| Sensor | ROS Topic | Rate | Encoding | Status |
|--------|-----------|------|----------|--------|
| LiDAR | `/scan` | 25Hz | LaserScan | Working |
| IMU | `/imu` | ~100Hz | Imu | Working |
| RGB Camera | `/camera/color/image_raw` | ~2Hz | rgb8 | Working (visible in RViz + rqt) |
| Depth Camera | `/camera/depth` | ~3Hz | 32FC1 | Working (valid data, RViz display cosmetic issue) |
| Odometry | `/odom` | ~100Hz | Odometry | Working |
| Clock | `/clock` | ~1kHz | Clock | Working |

### TF Tree

```
map → odom → base_link → chassis_link → lidar_link
(static)  (EKF)   (rsp)           → camera_link → camera_color_optical_frame
                                                 → camera_depth_optical_frame
                                   → suspension_axle → imu_link
                                                     → front_assembly → ...wheels
                                   → left_motor → back_left_wheel
                                   → right_motor → back_right_wheel
```

### Key Lessons from Phase 1

1. **Leaked GPU contexts:** Multiple crashed Gazebo Ignition sessions on dual-GPU systems leak rendering contexts that persist until reboot. Always reboot after Gazebo crashes.
2. **Fortress topic naming:** Sensors publish on `<topic>` directly, not `<topic>/image` or `<topic>/depth_image`. Check with `ign topic -l` to verify.
3. **LIBGL_ALWAYS_SOFTWARE:** NVIDIA drivers reject this env var (`Not allowed to force software rendering when API explicitly selects a hardware device`). Cameras use hardware rendering regardless.
4. **32FC1 depth in RViz:** RViz Image display cannot handle `inf` values in float depth. Use `rqt_image_view` for visualization.
5. **Camera toggle:** The `enable_cameras` xacro arg is essential -- cameras add rendering load and can cause issues. Keep disabled when not needed.

### Commits (Phase 1)

| Hash | Message |
|------|---------|
| `15f5f10` | Fix: Split rgbd_camera into separate camera + depth_camera sensors |
| `2aeceda` | Fix: Add enable_cameras toggle + software rendering for dual-GPU systems |
| `ec39321` | Fix: Only set LIBGL_ALWAYS_SOFTWARE when cameras enabled, restore gui=true |
| `e7d8bd6` | Update build log with fix 4 failure analysis and fix 5 |
| `4040c3d` | Checkpoint 1 passed: Gazebo sim fully functional after GPU state fix |
| `38c50e2` | Fix: Correct camera bridge topic names for Fortress |
| `5698526` | Checkpoint 1 fully complete: cameras publishing RGB + depth |
| `6a4a8e6` | Fix: RViz fixed frame odom (not map), add camera image displays |
| `a23eaaa` | Fix: Add static map->odom TF, restore RViz fixed frame to map |

---

## Phase 2: Depth Error Injector (2026-03-09)

### Goal

Write a ROS2 node that subscribes to GT depth from Gazebo (`/camera/depth`) and publishes error-injected depth (`/camera/depth_injected`) using one of 10 error profiles derived from real corridor evaluation data. This enables controlled experiments measuring how navigation performance degrades with depth quality.

### Files Created/Modified

| File | Action | Description |
|------|--------|-------------|
| `scripts/depth_error_injector.py` | **Created** | ROS2 node: subscribes to GT depth, applies depth-bin-specific Gaussian noise + dead pixel mask, publishes injected depth. 10 profiles, runtime-switchable via `ros2 param set`. |
| `CMakeLists.txt` | **Modified** | Added `install(PROGRAMS scripts/depth_error_injector.py ...)` to install the script as an executable. |
| `package.xml` | **Modified** | Added `sensor_msgs` and `cv_bridge` exec dependencies. |
| `launch/fortress_bringup.launch.py` | **Modified** | Added `depth_profile` launch argument (-1=disabled, 0-9=profile). Injector node conditionally launched when `depth_profile >= 0`. |

### Error Profile Design

10 profiles derived from 459-frame corridor benchmark (rosbag `rgbd_imu_20260228_003828_0.mcap`):

| ID | Name | Near RMSE (0.3-1m) | Mid RMSE (1-2m) | Far RMSE (2-4m) | Dead % | Source |
|----|------|---------------------|------------------|------------------|--------|--------|
| 0 | GT | 0.0 | 0.0 | 0.0 | 0% | Pass-through |
| 1 | DA3-Small | 0.158 | 0.486 | 1.320 | 0% | eval_corridor_depth.py |
| 2 | V4 | 1.434 | 1.182 | 1.283 | 0% | eval_corridor_depth.py |
| 3 | V5 | 2.361 | 2.0 (est) | 1.469 | 0% | eval_corridor_depth.py + interp |
| 4 | V6 | 2.262 | 2.0 (est) | 1.5 (est) | 0% | eval_corridor_depth.py + interp |
| 5 | V7 | 1.982 | 1.453 | 0.732 | 0% | eval_corridor_depth.py |
| 6 | V8 | 2.368 | 2.1 (est) | 1.6 (est) | 0% | changelog.tex + interp |
| 7 | V9 | 1.782 | 1.3 (est) | 1.0 (est) | 0% | changelog.tex + interp |
| 8 | sensor-fail | 0.0 | 0.0 | 0.0 | 77% | Femto Bolt dead pixel ratio |
| 9 | DA3+sensor-fail | 0.158 | 0.486 | 1.320 | 77% | DA3 noise + sensor dead pixels |

### Noise Model

Per-pixel depth-dependent additive Gaussian noise:

```
d_injected = d_gt + N(0, sigma(d_gt))
sigma(d) = near_rmse   if 0.3 <= d < 1.0
         = mid_rmse    if 1.0 <= d < 2.0
         = far_rmse    if 2.0 <= d < 4.0
         = beyond_rmse if d >= 4.0
         = near_rmse/2 if d < 0.3
```

Dead pixel mask uses 8x8 block-level Bernoulli sampling for spatial correlation (real sensor dead pixels cluster around glass/specular surfaces). Mask is deterministic per-seed for reproducibility.

### Architecture

```
/camera/depth (GT, 32FC1) ---> [depth_error_injector] ---> /camera/depth_injected (32FC1)
                                    |
                            profile param (0-9)
                            runtime switchable
```

### Checkpoint 2 Results (2026-03-09)

| Check | Expected | Actual | Status |
|-------|----------|--------|--------|
| `/camera/depth_injected` rate | ~2-3 Hz | 3.0 Hz | PASS |
| Image format | 320x240, 32FC1 | 320x240, 32FC1 | PASS |
| Profile 0 switch | Logs confirmation | Confirmed | PASS |
| Profile 1 switch | Logs "DA3-Small" | Confirmed | PASS |
| Profile 8 switch | Logs "sensor-fail" | Confirmed | PASS |
| Profile 8 dead pixels | ~77% | 85.2% (77% mask + 37% GT inf = 85% union) | PASS |
| Valid depth range (profile 8) | 0.3-8.5m | 0.320m-8.496m | PASS |

### Commits (Phase 2)

| Hash | Message |
|------|---------|
| `a960d10` | Phase 2: Add depth_error_injector.py with 10 error profiles |

### Launch Usage

```bash
# Without injector (default):
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf enable_cameras:=true

# With GT pass-through (profile 0):
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf enable_cameras:=true depth_profile:=0

# With DA3-Small noise (profile 1):
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf enable_cameras:=true depth_profile:=1

# Switch profile at runtime:
ros2 param set /depth_error_injector profile 3
```
