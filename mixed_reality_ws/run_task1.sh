#!/bin/bash
# Mixed-Reality Swarm Yielding Test - Quick Setup & Run
# Task 1: Prove physical robot yields to simulated robot's tether

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Mixed-Reality Swarm Yielding Test${NC}"
echo -e "${GREEN}  Task 1: Virtual Robot Tether Crossing${NC}"
echo -e "${GREEN}========================================${NC}\n"

# 1. Source ROS 2
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# 2. Build mixed_reality_ws
WS_PATH="/home/chandan/mixed_reality_ws"
echo "Building workspace: $WS_PATH"
cd "$WS_PATH"
colcon build --packages-select swarm_tether_estimator unitree_aliengo_bridge tether_detection 2>&1 | tail -20

# 3. Source workspace
source install/setup.bash

# 4. Check Unitree connection
echo -e "\n${YELLOW}Checking Unitree AlienGo connection...${NC}"
if ip link show | grep -q "can0"; then
    echo "✓ CAN interface (can0) detected"
else
    echo -e "${RED}⚠ Warning: can0 not found. Is the Unitree USB-CAN adapter connected?${NC}"
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 5. Launch experiment
echo -e "\n${GREEN}Starting mixed-reality experiment...${NC}"
echo "Virtual robots will cross physical robot's path."
echo "The physical Unitree should STOP and WAIT for tether to pass.\n"
echo "Press Ctrl+C to stop.\n"

ros2 launch unitree_aliengo_bridge mixed_reality_experiment.launch.py

# After exit
echo -e "\n${GREEN}Experiment ended.${NC}"
echo "Check velocity log at: ~/mixed_reality_data/velocity_log_*.csv"
echo "Run analysis: python3 ~/mixed_reality_ws/analyze_data.py"
