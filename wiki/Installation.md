# Installation Guide

This guide covers the setup process for the **Unified Navigation Framework for Tethered Robots**.

## Prerequisites

*   **Operating System**: Ubuntu 20.04 (Focal Fossa) recommended for ROS Noetic.
*   **ROS Version**: ROS Noetic Ninjemys (Desktop Full Install).
*   **Python**: Python 3.8+.

## 1. Install ROS Noetic

If you haven't installed ROS yet, follow the [official ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu) or run:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Initialize rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## 2. Clone the Repository

Create a workspace (if you don't have one) and clone the repo:

```bash
mkdir -p ~/atlas_ws/src
cd ~/atlas_ws/src
git clone https://github.com/Chandan118/Unified-Navigation-Framework-for-Tethered-Robots.git
```

## 3. Install Dependencies

Use `rosdep` to automatically install system dependencies:

```bash
cd ~/atlas_ws
rosdep install --from-paths src --ignore-src -r -y
```

Python dependencies:

```bash
pip3 install scikit-fuzzy numpy opencv-python
```

## 4. Build the Project

We use `catkin_make` (for ROS 1) to build the workspace:

```bash
cd ~/atlas_ws
catkin_make
source devel/setup.bash
```

## 5. (Optional) Docker Setup

If you prefer using Docker, a `Dockerfile` is provided in the root directory.

```bash
docker build -t atlas_nav_framework .
docker run -it --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" atlas_nav_framework
```

---
[Return to Home](Home.md)
