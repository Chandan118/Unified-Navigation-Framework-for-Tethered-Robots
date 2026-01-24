# Use official ROS Noetic image
FROM osrf/ros:noetic-desktop-full

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-velodyne-simulator \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-tf \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    scikit-fuzzy \
    opencv-python \
    noise \
    skimage

# Create workspace
WORKDIR /catkin_ws
RUN mkdir -p src

# Entrypoint setup
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
