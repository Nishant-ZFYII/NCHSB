# ROS2 Ackermann Bot

High-speed autonomous navigation system for an Ackermann-steered RC car in narrow corridors.

## Quick Start

```bash
# Build
colcon build --symlink-install
source install/setup.bash

# Run simulation
ros2 launch rc_model_description fortress_bringup.launch.py world:=track_30mps.world
```

## Tech Stack

- ROS2 Humble
- Gazebo Harmonic
- Nav2 (MPPI Controller)
- SLAM Toolbox
