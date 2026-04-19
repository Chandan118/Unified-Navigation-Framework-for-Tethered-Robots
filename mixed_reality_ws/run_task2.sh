#!/bin/bash
# Task 2: Physical Tether Dynamic Avoidance
# Unitree AlienGo uses LiDAR to detect and avoid a real rope

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Task 2: Physical Tether Avoidance${NC}"
echo -e "${GREEN}========================================${NC}\n"

echo -e "${YELLOW}Setup Instructions:${NC}"
echo "1. Attach a visible rope/line across the room (height ~10-20cm)"
echo "2. Have someone slowly move the rope across the robot's planned path"
echo "   OR tie it to an RC car that moves autonomously"
echo "3. Place the Unitree AlienGo at starting position"
echo "4. Give it a navigation goal on the opposite side of the room"
echo "5. The robot should detect the rope with LiDAR and reroute around it\n"

read -p "Is the physical tether setup ready? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please set up the physical tether and re-run this script."
    exit 1
fi

# Source ROS 2
source /opt/ros/humble/setup.bash
WS_PATH="/home/chandan/mixed_reality_ws"
cd "$WS_PATH"
source install/setup.bash

# Launch: tether detector + Unitree bridge
echo -e "\n${GREEN}Starting Task 2: LiDAR-based tether detection${NC}"
echo "Point cloud from RealSense/LiDAR → line extraction → avoidance"
echo "Data logged to: ~/mixed_reality_data/\n"
echo "Press Ctrl+C to stop.\n"

ros2 launch tether_detection tether_detector_node

echo -e "\n${GREEN}Task 2 complete.${NC}"
echo "Analyzing trajectory and Jetson latency..."
python3 "$WS_PATH/analyze_data.py"
