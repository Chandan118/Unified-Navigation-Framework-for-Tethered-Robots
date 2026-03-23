#!/bin/bash
# File        : ros_entrypoint.sh
# Author      : Chandan Sheikder
# Email       : chandan@bit.edu.cn
# Phone       : +8618222390506
# Affiliation : Beijing Institute of Technology (BIT)
# Date        : 2026-03-23
#
set -e

# Source ROS 2 Humble base
source "/opt/ros/humble/setup.bash"

# Source the workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source "/ros2_ws/install/setup.bash"
fi

exec "$@"
