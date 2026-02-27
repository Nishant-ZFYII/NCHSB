# rc_hardware_bringup

Hardware bringup package for the Traxxas Maxx 4S Ackermann RC car.  
Replaces the Gazebo simulation in `NCHSB/` with real sensor drivers and a Teensy serial bridge.

## Hardware Stack

| Component | Model | ROS Topic |
|-----------|-------|-----------|
| Compute | Jetson Orin Nano 8GB | — |
| LiDAR | RPLiDAR A2M8 (10 Hz, 18 m, 360°) | `/scan` |
| Depth Camera | Orbbec Femto Bolt (ToF + RGB + IMU) | `/camera/depth/*`, `/camera/color/*`, `/camera/imu` |
| Motor Controller | VESC 6 MkVI (via Teensy) | — |
| Microcontroller | Teensy 4.1 (USB serial) | `/odom`, `/joint_states`, `/steering_angle` |
| RC Override | ELRS 2.4 GHz | hardware-level at Teensy firmware |
| Encoders | 2× AS5600 magnetic | fed through Teensy |

## Sim vs. Hardware Mapping

| Aspect | NCHSB (Sim) | This Package (Hardware) |
|--------|-------------|------------------------|
| Physics | Gazebo Harmonic | Real world |
| LiDAR | GPU LiDAR plugin (25 Hz, 8 m) | RPLiDAR A2M8 (10 Hz, 18 m) |
| IMU | Gazebo IMU plugin | Femto Bolt IMU |
| Odometry | `ackermann_steering_controller` | Teensy bridge (AS5600 encoders) |
| Motor control | ros2_control → Gazebo joints | Teensy → VESC 6 MkVI |
| Clock | `/clock` from Gazebo | Wall clock (`use_sim_time: false`) |
| Depth | None | Femto Bolt ToF + student model fusion |
| URDF | `gazebo_control.xacro` included | `rc_model_hardware.urdf.xacro` (no Gazebo plugins) |

## Prerequisites

```bash
# ROS 2 Humble packages
sudo apt install ros-humble-rplidar-ros ros-humble-robot-localization \
    ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard

# Orbbec SDK (follow https://github.com/orbbec/OrbbecSDK_ROS2)
# Install orbbec_camera package from source

# Python deps
pip install pyserial
```

## Build

```bash
cd ~/MS_Project
colcon build --packages-select rc_hardware_bringup --symlink-install
source install/setup.bash
```

## USB Device Setup

Create udev rules so device paths are stable:

```bash
# /etc/udev/rules.d/99-rc-hardware.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="ttyTeensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyRPLiDAR"
```

Then use `serial_port_teensy:=/dev/ttyTeensy` and `serial_port_lidar:=/dev/ttyRPLiDAR`.

## Launch

### Step 1: Teleop Test (no navigation)

Verify sensors and motor control work before enabling autonomous navigation:

```bash
ros2 launch rc_hardware_bringup teleop_hardware.launch.py \
    serial_port_lidar:=/dev/ttyUSB0 \
    serial_port_teensy:=/dev/ttyACM0
```

Drive with keyboard (`i`/`,`/`j`/`l`/`k`). Verify in RViz:
- `/scan` shows LiDAR point cloud
- TF tree `odom → base_link` updates as car moves
- `/camera/color/image_raw` shows RGB feed

### Step 2: SLAM Mapping

Build a map of the corridor:

```bash
# Terminal 1: sensors
ros2 launch rc_hardware_bringup sensors_bringup.launch.py

# Terminal 2: SLAM in mapping mode
ros2 launch rc_hardware_bringup navigation_bringup.launch.py \
    slam_params:=$(ros2 pkg prefix rc_hardware_bringup)/share/rc_hardware_bringup/config/slam_hardware.yaml

# Terminal 3: teleop to drive around
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/corridor_map
```

### Step 3: Full Autonomous Navigation

```bash
ros2 launch rc_hardware_bringup hardware_bringup.launch.py \
    serial_port_lidar:=/dev/ttyUSB0 \
    serial_port_teensy:=/dev/ttyACM0
```

Send goals via RViz "2D Goal Pose" button.

## Architecture

```
RPLiDAR ──→ /scan ──→ SLAM Toolbox ──→ map→odom TF ──┐
                                                       ├──→ Nav2 ──→ /cmd_vel ──→ Teensy Bridge ──→ USB ──→ Teensy ──→ VESC/Servo
Femto Bolt IMU ──→ /camera/imu ──→ EKF ──→ odom→base  ┘                                                       ↑
Teensy ──→ /odom (encoders) ──→ EKF ──┘                                                                        │
                                                                                                                │
Femto Bolt ToF ──→ /camera/depth ──→ Depth Fusion ──→ /depth/fused ──→ Costmap                                 │
Femto Bolt RGB ──→ /camera/color ──→ YOLO ──→ class-aware inflation ──→ Costmap                                 │
                                                                                                                │
Teensy encoder feedback ────────────────────────────────────────────────────────────────────────────────────────┘
```

## Teensy Serial Protocol

**Jetson → Teensy:**
```
CMD <speed_mps> <steering_rad>\n
```

**Teensy → Jetson:**
```
ODOM <x> <y> <theta> <vx> <vyaw> <steering_rad>\n
```

The Teensy firmware handles RC override independently — when RC stick deflection
exceeds a threshold, the Teensy ignores all serial commands.

## Vehicle Parameters

See `params/vehicle.ini` for the full parameter set, compatible with the
[Raceline-Optimization](https://github.com/CL2-UWaterloo/Raceline-Optimization)
trajectory planner format.

| Parameter | Value |
|-----------|-------|
| Wheelbase | 0.1869 m |
| Front track | 0.137 m |
| Rear track | 0.145 m |
| Wheel radius | 0.055 m |
| Max steering angle | ±28° (0.4887 rad) |
| Min turn radius | 0.35 m |
| Max speed (indoor) | 10 m/s |
| Mass (loaded) | ~4.5 kg |

## Safety

- **Always** have the ELRS RC transmitter in hand when the car is powered
- Test the kill switch before every autonomous run
- Start at 0.3 m/s; increase only after multiple successful runs
- The `cmd_timeout_sec` parameter (default 0.5 s) in the Teensy bridge
  automatically stops the car if no commands are received
