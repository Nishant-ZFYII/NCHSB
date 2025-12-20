# Installation Guide

## System Requirements

- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **RAM:** 8 GB minimum, 16 GB recommended
- **GPU:** NVIDIA GPU recommended for Gazebo rendering
- **Storage:** 5 GB free space

## Step 1: Install ROS2 Humble

Follow the official installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 2: Install Gazebo and ros_gz

```bash
sudo apt install ros-humble-ros-gz -y
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge -y
```

## Step 3: Install Navigation and Control Packages

```bash
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-ackermann-steering-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard
```

## Step 4: Clone and Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/NCHSB.git
cd NCHSB

# Install rosdep dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Step 5: Verify Installation

```bash
# Check packages are installed
ros2 pkg list | grep rc_

# Should output:
# rc_model_description
# rc_nav_bridge
```

## Troubleshooting

### Gazebo not starting
```bash
# Kill any existing Gazebo processes
pkill -f "gz sim|ign gazebo"

# Check Gazebo version
gz sim --version
```

### Controller not spawning
```bash
# List active controllers
ros2 control list_controllers -c /controller_manager

# Check controller manager is running
ros2 node list | grep controller_manager
```

### Missing dependencies
```bash
# Reinstall rosdeps
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

