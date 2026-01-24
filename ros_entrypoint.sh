#!/bin/bash
set -e

# Source ROS and workspace
source "/opt/ros/noetic/setup.bash"
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

exec "$@"
