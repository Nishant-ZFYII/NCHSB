# System Architecture

## Overview

This document describes the complete system architecture for the high-speed narrow corridor autonomous racing platform.

## 1. Hardware Layer (Simulated)

### Robot Platform
- **Base:** Modified Traxxas RC Car
- **Wheelbase:** 0.1869 m
- **Track Width:** 0.137 m (front), 0.145 m (rear)
- **Wheel Radius:** 0.055 m
- **Drive:** Rear-wheel drive
- **Steering:** Ackermann front steering

### Sensors
| Sensor | Type | Rate | Range |
|--------|------|------|-------|
| LiDAR | 2D GPU LiDAR | 25 Hz | 8 m, 360° |
| IMU | 6-DOF | 50 Hz | N/A |
| Wheel Encoders | Quadrature | 100 Hz | N/A |

## 2. Perception Layer

### SLAM Toolbox
- **Mode:** Online Async / Localization
- **Solver:** Ceres (SPARSE_NORMAL_CHOLESKY)
- **Resolution:** 5 cm/cell
- **Loop Closure:** Enabled

### Extended Kalman Filter
- **Package:** robot_localization
- **Inputs:** Wheel odometry + IMU
- **Outputs:** Filtered pose, odom→base_link TF
- **Rate:** 50 Hz

## 3. Planning Layer

### Global Planner: Smac Hybrid A*
- Considers Ackermann constraints
- Plans in SE2 (x, y, θ) space
- Respects minimum turn radius

### Local Planner: MPPI Controller
- **Samples:** 2000 trajectories
- **Horizon:** 56 steps (2.8 s)
- **Rate:** 20 Hz
- **Motion Model:** Ackermann

### Costmaps
- **Global:** Full map, 2 Hz update
- **Local:** 4m × 4m rolling window, 10 Hz update
- **Layers:** Static, Obstacle, Inflation

## 4. Control Layer

### Ackermann Steering Controller
- **Rate:** 100 Hz
- **Input:** TwistStamped (vx, wz)
- **Output:** Steering angles + wheel velocities
- **Odometry:** Published to /odom

### Joint Configuration
| Joint | Type | Command |
|-------|------|---------|
| left_wheel_mount_turn | Revolute | Position |
| right_wheel_mount_turn | Revolute | Position |
| back_left_wheel_joint | Continuous | Velocity |
| back_right_wheel_joint | Continuous | Velocity |

## 5. TF Tree

```
map
 └── odom (SLAM Toolbox)
      └── base_link (EKF)
           ├── chassis_link
           │    ├── lidar_link
           │    ├── suspension_axle
           │    │    ├── front_assembly
           │    │    │    ├── left_wheel_mount
           │    │    │    │    └── front_left_wheel
           │    │    │    └── right_wheel_mount
           │    │    │         └── front_right_wheel
           │    │    └── imu_link
           │    ├── left_motor
           │    │    └── back_left_wheel
           │    └── right_motor
           │         └── back_right_wheel
```

## 6. Topic Graph

### Sensor Topics
- `/scan` - LaserScan (LiDAR)
- `/imu` - Imu (IMU)
- `/odom` - Odometry (wheel encoder)

### Navigation Topics
- `/map` - OccupancyGrid
- `/odometry/filtered` - Filtered pose
- `/cmd_vel` - Velocity commands
- `/plan` - Global path

### Control Topics
- `/ackermann_steering_controller/reference` - TwistStamped commands
- `/joint_states` - Joint positions/velocities

## 7. Timing Diagram

```
Time →
├────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤
     10ms      20ms      30ms      40ms      50ms

LiDAR:    ████████████████████████████████████████  (40ms cycle)
IMU:      ██    ██    ██    ██    ██    ██    ██    (20ms cycle)
EKF:      ██    ██    ██    ██    ██    ██    ██    (20ms cycle)
MPPI:               ██████████████████              (50ms cycle)
Control:  █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █  (10ms cycle)
```

## 8. Data Flow

```
LiDAR ──→ SLAM ──→ map→odom TF ──┐
                                  ├──→ Nav2 ──→ cmd_vel ──→ Stamper ──→ Controller ──→ Wheels
IMU ────→ EKF ──→ odom→base_link ┘                              ↑
Wheels ──┘                                                       │
    └────────────────────────────────────────────────────────────┘
                            (feedback loop)
```

