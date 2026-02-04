#!/bin/bash
# Installation and Build Script for ATLAS-T Simulation

set -e  # Exit on error

echo "======================================"
echo "ATLAS-T Simulation Setup Script"
echo "======================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS installation
echo -e "${YELLOW}Checking ROS installation...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ROS not found! Please install ROS Noetic first.${NC}"
    echo "Visit: http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

if [ "$ROS_DISTRO" != "noetic" ]; then
    echo -e "${YELLOW}Warning: ROS $ROS_DISTRO detected. This project is designed for ROS Noetic.${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "${GREEN}ROS $ROS_DISTRO found!${NC}"

# Install ROS dependencies
echo -e "${YELLOW}Installing ROS dependencies...${NC}"
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-xacro \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    ros-noetic-rqt-plot \
    python3-pip

echo -e "${GREEN}ROS dependencies installed!${NC}"

# Install Python dependencies
echo -e "${YELLOW}Installing Python dependencies...${NC}"
pip3 install --user numpy scipy scikit-fuzzy opencv-python noise

echo -e "${GREEN}Python dependencies installed!${NC}"

# Navigate to workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Generate world file
echo -e "${YELLOW}Generating procedural world...${NC}"
if [ -f "src/atlas_gazebo/scripts/world_generator.py" ]; then
    python3 src/atlas_gazebo/scripts/world_generator.py src/atlas_gazebo/worlds/warehouse_world.world
    echo -e "${GREEN}World generated!${NC}"
else
    echo -e "${YELLOW}World generator not found, skipping...${NC}"
fi

# Build workspace
echo -e "${YELLOW}Building ROS workspace...${NC}"
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
else
    echo -e "${RED}Build failed! Check errors above.${NC}"
    exit 1
fi

# Source workspace
source devel/setup.bash

echo ""
echo -e "${GREEN}======================================"
echo "Setup Complete!"
echo "======================================${NC}"
echo ""
echo "To use the workspace, run:"
echo "  source devel/setup.bash"
echo ""
echo "To launch the simulation:"
echo "  roslaunch hybrid_navigation master.launch"
echo ""
echo "To test individual components:"
echo "  roslaunch atlas_gazebo spawn_robot.launch"
echo "  roslaunch hybrid_navigation hybrid_navigation.launch"
echo ""
