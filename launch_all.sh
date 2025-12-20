#!/bin/bash
# =============================================================================
# NCHSB Full System Launch Script
# Launches all components in the correct order with proper timing
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="/home/asas/Documents/NCHSB"
WORLD_NAME="default"  # Must match the world name in your .world file

# Function to print colored messages
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to wait for a ROS topic to be available
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local count=0
    print_status "Waiting for topic $topic..."
    while ! ros2 topic list 2>/dev/null | grep -q "^${topic}$"; do
        sleep 1
        count=$((count + 1))
        if [ $count -ge $timeout ]; then
            print_warning "Timeout waiting for topic $topic"
            return 1
        fi
    done
    print_status "Topic $topic is available"
    return 0
}

# Function to unpause Gazebo simulation
unpause_gazebo() {
    print_status "Unpausing Gazebo simulation..."
    sleep 2  # Give Gazebo a moment to fully initialize
    
    # Try gz command first (Gazebo Fortress/Garden)
    if command_exists gz; then
        gz service -s /world/${WORLD_NAME}/control \
            --reqtype gz.msgs.WorldControl \
            --reptype gz.msgs.Boolean \
            --timeout 3000 \
            --req 'pause: false' 2>/dev/null && {
            print_status "Gazebo unpaused successfully (gz)"
            return 0
        }
    fi
    
    # Try ign command (older Ignition Gazebo)
    if command_exists ign; then
        ign service -s /world/${WORLD_NAME}/control \
            --reqtype ignition.msgs.WorldControl \
            --reptype ignition.msgs.Boolean \
            --timeout 3000 \
            --req 'pause: false' 2>/dev/null && {
            print_status "Gazebo unpaused successfully (ign)"
            return 0
        }
    fi
    
    print_warning "Could not unpause Gazebo automatically. Please press Play manually."
    return 1
}

# Cleanup function
cleanup() {
    print_warning "Shutting down all processes..."
    # Kill all background jobs
    jobs -p | xargs -r kill 2>/dev/null
    # Kill any remaining ROS processes from this script
    pkill -P $$ 2>/dev/null || true
    print_status "Cleanup complete"
    exit 0
}

# Set up trap for cleanup on script exit
trap cleanup SIGINT SIGTERM

# =============================================================================
# MAIN SCRIPT
# =============================================================================

echo ""
echo "=========================================="
echo "  NCHSB Full System Launch"
echo "=========================================="
echo ""

# Check if workspace is sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    print_status "Sourcing ROS 2 workspace..."
    source /opt/ros/humble/setup.bash
    source ${WORKSPACE_DIR}/install/setup.bash
fi

# Verify packages exist
print_status "Verifying packages..."
if ! ros2 pkg prefix rc_model_description >/dev/null 2>&1; then
    print_error "Package rc_model_description not found. Please build the workspace first:"
    echo "  cd ${WORKSPACE_DIR} && colcon build && source install/setup.bash"
    exit 1
fi

# -----------------------------------------------------------------------------
# Step 1: Launch Gazebo + Robot + Controllers
# -----------------------------------------------------------------------------
print_step "1/6 - Launching Gazebo simulation (fortress_bringup)..."
ros2 launch rc_model_description fortress_bringup.launch.py &
GAZEBO_PID=$!

# Wait for Gazebo to start and robot to spawn
print_status "Waiting for Gazebo to initialize..."
sleep 5

# Unpause Gazebo (press play button)
unpause_gazebo

# Wait for controllers to activate
print_status "Waiting for controllers to activate..."
sleep 3

# -----------------------------------------------------------------------------
# Step 2: Launch SLAM Localization
# -----------------------------------------------------------------------------
print_step "2/6 - Launching SLAM Toolbox (localization mode)..."
ros2 launch rc_model_description slam_localization.launch.py use_sim_time:=true &
SLAM_PID=$!
sleep 3

# -----------------------------------------------------------------------------
# Step 3: Launch EKF for sensor fusion
# -----------------------------------------------------------------------------
print_step "3/6 - Launching EKF (IMU + Odom fusion)..."
ros2 launch rc_model_description ekf_imu_odom.launch.py use_sim_time:=true &
EKF_PID=$!
sleep 2

# -----------------------------------------------------------------------------
# Step 4: Launch Nav2
# -----------------------------------------------------------------------------
print_step "4/6 - Launching Nav2 stack..."
ros2 launch rc_model_description nav2_rc_bringup.launch.py use_sim_time:=true &
NAV2_PID=$!
sleep 3

# -----------------------------------------------------------------------------
# Step 5: Launch cmd_vel bridge (stamper)
# -----------------------------------------------------------------------------
print_step "5/6 - Launching cmd_vel stamper bridge..."
ros2 run rc_nav_bridge stamper --ros-args \
    -p frame_id:=base_link \
    -r cmd_vel:=/cmd_vel \
    -r reference:=/ackermann_steering_controller/reference &
STAMPER_PID=$!
sleep 1

# -----------------------------------------------------------------------------
# Step 6: Launch Teleop (in foreground for keyboard control)
# -----------------------------------------------------------------------------
print_step "6/6 - Launching teleop keyboard..."
echo ""
echo "=========================================="
echo "  All systems launched!"
echo "=========================================="
echo ""
echo "  Use keyboard to control the robot:"
echo "    i - forward"
echo "    , - backward"
echo "    j - turn left"
echo "    l - turn right"
echo "    k - stop"
echo ""
echo "  Press Ctrl+C to shut down all processes"
echo "=========================================="
echo ""

# Run teleop in foreground (this blocks until user exits)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# When teleop exits, cleanup
cleanup

