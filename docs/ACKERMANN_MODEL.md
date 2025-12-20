# Ackermann Steering Model

## Overview

This document describes the Ackermann steering model implementation, including the URDF challenges and solutions.

## 1. Why Ackermann Steering?

### Comparison with Differential Drive

| Aspect | Differential Drive | Ackermann Steering |
|--------|-------------------|-------------------|
| Turn in place | ✅ Yes | ❌ No |
| High-speed stability | ❌ Poor (>2 m/s) | ✅ Excellent |
| Trajectory smoothness | ❌ Discontinuous | ✅ C² continuous |
| Real vehicle match | ❌ Robot-specific | ✅ Car-like |

### Requirements Driving Ackermann Choice

1. **High-speed operation (6-30 m/s)** - Differential drive becomes unstable
2. **Smooth trajectories** - Required for narrow corridor racing
3. **Real hardware match** - Target is a Traxxas RC car

## 2. The Four-Bar Linkage Problem

### Real Ackermann Mechanism

Real Ackermann steering uses a four-bar linkage (trapezoid) that creates a **closed kinematic loop**:

```
     ┌───────────────────────┐
     │     Steering Rod      │
     └───────┬───────┬───────┘
             │       │
        ┌────┴──┐ ┌──┴────┐
        │Tie Rod│ │Tie Rod│
        └───┬───┘ └───┬───┘
            │         │
       ┌────┴───┐ ┌───┴────┐
       │Kingpin │ │Kingpin │
       │ Left   │ │ Right  │
       └────────┘ └────────┘
            ↓         ↓
         ═══════════════════
              Chassis
              (fixed)
```

### URDF Limitation

> "URDF only supports **tree structures** (parent→child), NOT closed loops."

The steering linkage creates a loop that URDF cannot represent.

## 3. Solution: Mimic Joints

### Kinematic Observation

In Ackermann steering, the servo horn and wheel mounts share approximately the same rotation angle.

### Implementation

```xml
<!-- Master joint -->
<joint name="servo_angle" type="revolute">
    <parent link="servo" />
    <child link="servo_horn" />
    <axis xyz="0 0 -1" />
    <limit upper="0.488692" lower="-0.488692" />
</joint>

<!-- Left wheel mimics servo -->
<joint name="left_wheel_mount_turn" type="revolute">
    <parent link="front_assembly" />
    <child link="left_wheel_mount" />
    <mimic joint="servo_angle" multiplier="-1" />
</joint>

<!-- Right wheel mimics servo -->
<joint name="right_wheel_mount_turn" type="revolute">
    <parent link="front_assembly" />
    <child link="right_wheel_mount" />
    <mimic joint="servo_angle" multiplier="-1" />
</joint>
```

## 4. Current Implementation: Direct Control

In the final implementation, we use **direct control** instead of mimic joints:

1. Both steering joints are independently controllable
2. The Ackermann controller computes correct angles
3. This provides **true Ackermann geometry** (inner wheel turns more)

### Controller Configuration

```yaml
ackermann_steering_controller:
  ros__parameters:
    wheelbase: 0.1869
    front_wheel_track: 0.137
    rear_wheel_track: 0.145
    front_wheels_radius: 0.055
    rear_wheels_radius: 0.055
    
    front_wheels_names: [left_wheel_mount_turn, right_wheel_mount_turn]
    rear_wheels_names: [back_left_wheel_joint, back_right_wheel_joint]
```

## 5. Kinematic Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Wheelbase (L) | 0.1869 m | CAD |
| Front Track | 0.137 m | CAD |
| Rear Track | 0.145 m | CAD |
| Wheel Radius | 0.055 m | CAD |
| Max Steering Angle | ±28° (0.488 rad) | Joint limits |
| Min Turn Radius | 0.35 m | Computed |

### Minimum Turn Radius Calculation

```
R_min = L / tan(δ_max)
      = 0.1869 / tan(0.488692)
      = 0.1869 / 0.532
      = 0.35 m
```

## 6. Ackermann Angle Computation

Given desired turning radius R:

```
δ_inner = atan(L / (R - T/2))  # Inner wheel turns MORE
δ_outer = atan(L / (R + T/2))  # Outer wheel turns LESS
```

### Example

For R = 2.0 m:
- δ_inner = atan(0.1869 / (2.0 - 0.0685)) = 0.0968 rad = 5.5°
- δ_outer = atan(0.1869 / (2.0 + 0.0685)) = 0.0903 rad = 5.2°

## 7. Validation Results

| Test | Expected | Measured | Error |
|------|----------|----------|-------|
| Turn radius (R=2.0m) | 2.00 m | 1.98 m | 1% |
| Min turn radius | 0.35 m | 0.38 m | 8% |
| Max steering angle | 28° | 28° | 0% |

## 8. Integration with Path Planner

The minimum turn radius constrains the path planner:

```yaml
# MPPI Controller constraint
AckermannConstraints:
  min_turning_r: 0.30  # Allows slight margin
```

Trajectories requiring tighter turns are automatically rejected by the MPPI sampling process.

