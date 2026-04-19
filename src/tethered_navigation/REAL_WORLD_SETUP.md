# Real-World Stabilization and Training Guide

This guide is for your robot with:
- Jetson Orin Nano 8GB
- Arduino Nano motor controller
- IMU, 2D LiDAR, camera, ultrasonic sensor

## 1) What changed in this package

- Added `safety_cmd_vel_filter.py`:
  - Reads planner commands from `/cmd_vel_nav`
  - Uses LiDAR + ultrasonic + camera-obstacle signal
  - Publishes safe commands to `/cmd_vel`
  - Enforces timeout and acceleration limits

- Added `camera_obstacle_detector.py`:
  - Runs YOLO (`yolov8n.pt` by default) on camera frames
  - Publishes:
    - `/camera/obstacle` (`std_msgs/Bool`)
    - `/camera/obstacle_score` (`std_msgs/Float32`)

- Updated bringup:
  - `robot_bringup.launch.py` now starts hardware + safety nodes
  - `navigate.launch.py` now uses full `nav2_bringup` stack
  - Conservative Nav2 parameters in `config/nav2_params.yaml`

## 2) Build

```bash
cd ~/ros2_ws
colcon build --packages-select tethered_navigation
source install/setup.bash
```

## 3) Run navigation on real robot

```bash
ros2 launch tethered_navigation navigate.launch.py \
  map:=/home/jetson/formica_map.yaml \
  use_sim_time:=false
```

If your ports differ:

```bash
ros2 launch tethered_navigation robot_bringup.launch.py \
  arduino_port:=/dev/ttyUSB0 \
  imu_port:=/dev/ttyUSB1 \
  front_lidar_port:=/dev/ttyUSB2 \
  rear_lidar_port:=/dev/ttyUSB3 \
  camera_device:=/dev/video0
```

## 4) Safety checks before motion

1. Verify sensor topics:
   - `/scan`
   - `/sensor/ultrasonic`
   - `/camera/image_raw`
   - `/camera/obstacle`
2. Verify output flow:
   - planner publishes `/cmd_vel_nav`
   - filter publishes `/cmd_vel`
3. Ensure robot stops if `/cmd_vel_nav` pauses.

## 5) Camera training workflow (real-world data)

You should train the detector with your own environment images.

### 5.1 Collect images

- Record different lighting and obstacle types in your actual workspace.
- Include negative samples (clear path) and positive samples (blocked path).
- Aim for at least 1500 labeled images.

### 5.2 Label

- Use [CVAT](https://www.cvat.ai/) or [Label Studio](https://labelstud.io/).
- Label obstacle classes relevant to your site (person, box, chair, wall edge, etc).

### 5.3 Train YOLO

```bash
pip3 install -U ultralytics
yolo detect train \
  model=yolov8n.pt \
  data=/path/to/dataset.yaml \
  imgsz=640 \
  epochs=60 \
  batch=16 \
  device=0 \
  name=robot_obstacle_v1
```

### 5.4 Deploy

Set trained model:

```bash
ros2 run tethered_navigation camera_obstacle_detector.py \
  --ros-args -p model_path:=/path/to/best.pt
```

## 6) Tuning order

1. Motor direction and encoder correctness
2. IMU frame alignment
3. LiDAR TF alignment
4. EKF validation (`/odom` smooth and consistent)
5. Nav2 tuning (speed, inflation radius, recovery behavior)
6. Camera detector threshold tuning

Do not skip this order.
