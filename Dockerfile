# Start from official ROS 2 Humble base
FROM ros:humble-ros-base

# Set environment variables
ENV ROS_DISTRO=humble
ENV TERM=xterm-256color

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    scikit-fuzzy \
    numpy \
    opencv-python

# Create workspace
WORKDIR /ros2_ws
COPY ./src ./src

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Copy entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
