#!/bin/bash

# Robot Startup Script (tmux version)
# This script launches all necessary nodes using tmux

SESSION_NAME="robot_system"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Starting Robot System (tmux)${NC}"
echo -e "${GREEN}========================================${NC}"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}Error: tmux is not installed${NC}"
    echo -e "${BLUE}Install with: sudo apt install tmux${NC}"
    exit 1
fi

# Check if ROS2 workspace exists
if [ ! -d "$HOME/ros2_ws" ]; then
    echo -e "${RED}Error: ros2_ws not found in home directory${NC}"
    exit 1
fi

# Kill existing session if it exists
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${BLUE}Killing existing session...${NC}"
    tmux kill-session -t $SESSION_NAME
fi

# Create new tmux session
echo -e "${BLUE}Creating tmux session: $SESSION_NAME${NC}"
tmux new-session -d -s $SESSION_NAME -n "serial_odom"

# Window 1: Serial Odometry
tmux send-keys -t $SESSION_NAME:0 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0 "source $HOME/ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0 "echo '=== Serial Odometry Node ==='" C-m
tmux send-keys -t $SESSION_NAME:0 "python3 $HOME/ros2_ws/src/serial_odom_node/serial_odom_node.py" C-m

# Window 2: IMU Driver
tmux new-window -t $SESSION_NAME:1 -n "imu_driver"
tmux send-keys -t $SESSION_NAME:1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:1 "source $HOME/ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:1 "echo '=== IMU Driver ==='" C-m
tmux send-keys -t $SESSION_NAME:1 "ros2 launch icm20948_ros2_driver icm20948.launch.py" C-m

# Window 3: Robot Launch
tmux new-window -t $SESSION_NAME:2 -n "robot_system"
tmux send-keys -t $SESSION_NAME:2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:2 "source $HOME/ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:2 "echo '=== Robot System ==='" C-m
tmux send-keys -t $SESSION_NAME:2 "ros2 launch my_robot robot.launch.py" C-m

# Select first window
tmux select-window -t $SESSION_NAME:0

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Tmux session created successfully!${NC}"
echo -e "${BLUE}Attach to session with:${NC} tmux attach -t $SESSION_NAME"
echo -e "${BLUE}Detach from session:${NC} Ctrl+b then d"
echo -e "${BLUE}Switch windows:${NC} Ctrl+b then number (0,1,2)"
echo -e "${BLUE}Kill session:${NC} tmux kill-session -t $SESSION_NAME"
echo -e "${GREEN}========================================${NC}"

# Attach to the session
tmux attach -t $SESSION_NAME
