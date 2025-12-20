#!/bin/bash
# =============================================================================
# NCHSB Multi-Terminal Launch Script
# Opens each component in its own terminal window
# =============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="/home/asas/Documents/NCHSB"
WORLD_NAME="default"

# Array to store terminal PIDs
declare -a TERMINAL_PIDS=()

# Function to print colored messages
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Function to open a new terminal with a command
# Usage: open_terminal "Title" "command to run"
open_terminal() {
    local title=$1
    local cmd=$2
    
    # Build the full command with sourcing
    local full_cmd="cd ${WORKSPACE_DIR} && source /opt/ros/humble/setup.bash && source install/setup.bash && echo '=== ${title} ===' && ${cmd}; exec bash"
    
    # Try different terminal emulators
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="${title}" -- bash -c "${full_cmd}" &
        TERMINAL_PIDS+=($!)
    elif command -v xterm &> /dev/null; then
        xterm -title "${title}" -e "bash -c '${full_cmd}'" &
        TERMINAL_PIDS+=($!)
    elif command -v konsole &> /dev/null; then
        konsole --new-tab -e bash -c "${full_cmd}" &
        TERMINAL_PIDS+=($!)
    elif command -v xfce4-terminal &> /dev/null; then
        xfce4-terminal --title="${title}" -e "bash -c '${full_cmd}'" &
        TERMINAL_PIDS+=($!)
    else
        print_warning "No supported terminal emulator found. Running in background..."
        bash -c "cd ${WORKSPACE_DIR} && source /opt/ros/humble/setup.bash && source install/setup.bash && ${cmd}" &
        TERMINAL_PIDS+=($!)
    fi
    
    print_status "Opened: ${title} (PID: ${TERMINAL_PIDS[-1]})"
}

# Function to unpause Gazebo
unpause_gazebo() {
    print_status "Unpausing Gazebo simulation..."
    
    # Source ROS for gz command
    source /opt/ros/humble/setup.bash
    
    # Try gz command (Gazebo Fortress/Garden)
    if command -v gz &> /dev/null; then
        gz service -s /world/${WORLD_NAME}/control \
            --reqtype gz.msgs.WorldControl \
            --reptype gz.msgs.Boolean \
            --timeout 3000 \
            --req 'pause: false' 2>/dev/null && {
            print_status "Gazebo unpaused successfully"
            return 0
        }
    fi
    
    # Try ign command (older Ignition)
    if command -v ign &> /dev/null; then
        ign service -s /world/${WORLD_NAME}/control \
            --reqtype ignition.msgs.WorldControl \
            --reptype ignition.msgs.Boolean \
            --timeout 3000 \
            --req 'pause: false' 2>/dev/null && {
            print_status "Gazebo unpaused successfully"
            return 0
        }
    fi
    
    print_warning "Could not unpause Gazebo automatically. Please press Play manually."
    return 1
}

# Cleanup function - kills all spawned terminals and processes
cleanup() {
    echo ""
    print_warning "Shutting down all processes..."
    
    # Kill all spawned terminal windows
    for pid in "${TERMINAL_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            print_status "Killing terminal PID: $pid"
            kill -TERM "$pid" 2>/dev/null
        fi
    done
    
    # Kill any remaining ROS/Gazebo processes started by us
    # Using pkill with pattern matching
    sleep 1
    
    # Kill Gazebo processes
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    
    # Kill RViz
    pkill -f "rviz2" 2>/dev/null || true
    
    # Kill ROS nodes from our launches
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "ekf_node" 2>/dev/null || true
    pkill -f "controller_manager" 2>/dev/null || true
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "stamper" 2>/dev/null || true
    pkill -f "teleop_twist_keyboard" 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    
    print_status "All processes terminated"
    print_status "Cleanup complete!"
    exit 0
}

# Set up trap for cleanup
trap cleanup SIGINT SIGTERM

# =============================================================================
# MAIN SCRIPT
# =============================================================================

clear
echo ""
echo "=========================================="
echo "  NCHSB Multi-Terminal Launch"
echo "=========================================="
echo ""

# Check if workspace exists
if [ ! -d "${WORKSPACE_DIR}/install" ]; then
    echo -e "${RED}[ERROR]${NC} Workspace not built. Please run:"
    echo "  cd ${WORKSPACE_DIR} && colcon build"
    exit 1
fi

# -----------------------------------------------------------------------------
# Step 1: Launch Gazebo + Robot + Controllers
# -----------------------------------------------------------------------------
print_step "1/6 - Opening Gazebo simulation terminal..."
open_terminal "1-Gazebo+Robot" "ros2 launch rc_model_description fortress_bringup.launch.py"

# Wait for Gazebo to start
print_status "Waiting for Gazebo to initialize (8 seconds)..."
sleep 8

# Unpause Gazebo
unpause_gazebo

# Wait for controllers
print_status "Waiting for controllers (3 seconds)..."
sleep 3

# -----------------------------------------------------------------------------
# Step 2: Launch SLAM Localization
# -----------------------------------------------------------------------------
print_step "2/6 - Opening SLAM Toolbox terminal..."
open_terminal "2-SLAM-Localization" "ros2 launch rc_model_description slam_localization.launch.py use_sim_time:=true"
sleep 3

# -----------------------------------------------------------------------------
# Step 3: Launch EKF
# -----------------------------------------------------------------------------
print_step "3/6 - Opening EKF terminal..."
open_terminal "3-EKF" "ros2 launch rc_model_description ekf_imu_odom.launch.py use_sim_time:=true"
sleep 2

# -----------------------------------------------------------------------------
# Step 4: Launch Nav2
# -----------------------------------------------------------------------------
print_step "4/6 - Opening Nav2 terminal..."
open_terminal "4-Nav2" "ros2 launch rc_model_description nav2_rc_bringup.launch.py use_sim_time:=true"
sleep 3

# -----------------------------------------------------------------------------
# Step 5: Launch Stamper Bridge
# -----------------------------------------------------------------------------
print_step "5/6 - Opening Stamper bridge terminal..."
open_terminal "5-Stamper" "ros2 run rc_nav_bridge stamper --ros-args -p frame_id:=base_link -r cmd_vel:=/cmd_vel -r reference:=/ackermann_steering_controller/reference"
sleep 1

# -----------------------------------------------------------------------------
# Step 6: Launch Teleop
# -----------------------------------------------------------------------------
print_step "6/6 - Opening Teleop terminal..."
open_terminal "6-Teleop" "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"

# -----------------------------------------------------------------------------
# Done - Wait for user to quit
# -----------------------------------------------------------------------------
echo ""
echo "=========================================="
echo -e "  ${GREEN}All terminals launched!${NC}"
echo "=========================================="
echo ""
echo "  Terminals opened:"
echo "    1-Gazebo+Robot    - Simulation & robot"
echo "    2-SLAM-Localization - Map localization"
echo "    3-EKF             - Sensor fusion"
echo "    4-Nav2            - Navigation stack"
echo "    5-Stamper         - cmd_vel bridge"
echo "    6-Teleop          - Keyboard control"
echo ""
echo "  Use the Teleop terminal to control the robot"
echo ""
echo -e "  ${YELLOW}Press Ctrl+C here to close ALL terminals${NC}"
echo "=========================================="
echo ""

# Wait forever until Ctrl+C
while true; do
    sleep 1
done

