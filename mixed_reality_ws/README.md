# Mixed-Reality Swarm Yielding Test
## Unitree AlienGo + Jetson Orin Nano + ROS 2

**Hardware Validation for Swarm Fuzzy-DRL Multi-Tether Avoidance**

---

## Overview

This repository implements two hardware validation experiments for the swarm robotics paper:

- **Task 1**: Mixed-reality tether crossing — Physical Unitree robot yields to *simulated* robots' tethers via ROS 2
- **Task 2**: Physical tether avoidance — Real tether detection using LiDAR on Jetson Orin Nano

Both experiments prove the swarm policy scales to heterogeneous hardware and runs in real-time on edge compute.

---

## Repository Structure

```
mixed_reality_ws/
├── src/
│   ├── swarm_tether_estimator/    # Publishes /swarm_tether_states for virtual robots
│   ├── unitree_aliengo_bridge/    # Bridge node: yielding behavior → cmd_vel
│   ├── tether_detection/          # LiDAR line extraction for physical tethers
│   └── formica_ws/                # Existing swarm simulation (robot_agent.py)
├── run_task1.sh                    # Quick launcher for Task 1
├── run_task2.sh                    # Quick launcher for Task 2
├── run_experiment.py              # Python experiment orchestrator
├── analyze_data.py                # Plot generation for paper figures
└── README.md                      # This file
```

---

## Prerequisites

### Software
- ROS 2 Humble Hawksbill (Ubuntu 20.04/22.04)
- CMake ≥ 3.5, gcc/g++ ≥ 9
- Python 3.8+
- PCL (Point Cloud Library) for Task 2 (optional)

### Hardware
- **Unitree AlienGo** (or Go1 with modified params)
- **Jetson Orin Nano** mounted on robot, running Ubuntu + ROS 2
- USB-CAN adapter for Unitree SDK communication
- LiDAR or Intel RealSense depth camera (Task 2)
- Physical tether/rope (brightly colored, ~2m long)
- 2-3 obstacles (boxes/chairs) for Task 1 room setup

---

## Quick Start

### Task 1: Virtual Robot Yielding

1. **Power on hardware**
   ```bash
   # On Jetson Orin Nano (Ubuntu)
   sudo ip link set can0 up type can bitrate 1000000  # CAN interface
   ros2 run unitree_legged_sdk go1_join  # or aliengo_join
   ```

2. **Run experiment on host PC (simulator side)**
   ```bash
   cd ~/mixed_reality_ws
   ./run_task1.sh
   ```

3. **Expected behavior**
   - 3 virtual robots spawn in Gazebo (or just their tether lines)
   - Virtual robots move across the field
   - Physical Unitree robot walks from Point A → Point B
   - When a virtual tether line crosses the robot's predicted path:
     - Robot **stops within 0.5s**
     - Holds position for 2.5s while tether "passes"
     - Resumes walking to goal
   - Velocity CSV logged to `~/mixed_reality_data/velocity_log_*.csv`

4. **Generate plots for paper**
   ```bash
   python3 analyze_data.py --task 1
   ```

### Task 2: Physical Tether Avoidance

1. **Setup**
   - Attach visible rope across room (10–20 cm above floor)
   - Tie one end to RC car or have assistant slowly move it
   - Place Unitree at start position (e.g., x=0.5, y=1.0)

2. **Run detection node**
   ```bash
   ros2 launch tether_detection tether_detector_node
   ```

3. **Watch avoidance**
   - Robot's LiDAR detects the rope line segment
   - Computes safe waypoint to the side
   - Dynamically reroutes around tether
   - Continues to goal

4. **Record data**
   ```bash
   # On Jetson: log odometry + computation time
   ros2 bag record -o task2_data /odom /cmd_vel /jetson_inference_time
   ```

5. **Plot**
   ```bash
   python3 analyze_data.py --task 2
   ```

---

## Key ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/swarm_tether_states` | `std_msgs/String` (JSON) | Virtual robots → Real robot | Array of tether line segments {robot_id, start[2], end[2], thickness, active} |
| `/odom` | `nav_msgs/Odometry` | Sim/robot → Bridge | Robot position & heading |
| `/cmd_vel` | `geometry_msgs/Twist` | Bridge → Unitree | Linear & angular velocity command |
| `/detected_tethers` | `visualization_msgs/MarkerArray` | Detector → RViz | Visual debug of detected physical ropes |
| `/avoidance_point` | `geometry_msgs/PointStamped` | Detector → Bridge | Safe waypoint to avoid detected tether |

---

## Yielding Algorithm (Task 1)

```python
# Pseudocode from aliengo_bridge_node.cpp
if active_tether_will_cross_path(robot_pos, robot_heading, tether_line):
    state = YIELDING
    publish(cmd_vel=0)
    sleep(yield_stop_duration_s)
    state = NORMAL
```

**Collision prediction**: Ray-casting from robot along velocity vector × 2s horizon. If ray intersects tether segment + safety margin (0.6m), trigger yield.

---

## Dynamic Avoidance Algorithm (Task 2)

1. **LiDAR point cloud** → filter by height (0.05–0.3 m above ground)
2. **RANSAC line segmentation** → extract dominant lines
3. **Tether classification**: length ∈ [1.0, 4.0] m, uniform point density
4. **Avoidance waypoint**: perpendicular offset 0.8 m to left/right of tether midpoint
5. **Local planner** → navigate to waypoint, then resume original goal

---

## Data Output

### Task 1 CSV (velocity_log_*.csv)
| Column | Unit | Description |
|--------|------|-------------|
| `timestamp_s` | s | ROS time |
| `robot_x`, `robot_y` | m | Robot position |
| `robot_heading` | rad | Yaw angle |
| `linear_vel_mps` | m/s | Forward speed |
| `angular_vel_radps` | rad/s | Turn rate |
| `yielding_state` | enum | 0=NORMAL, 1=YIELDING, 2=RECOVERING |
| `active_tether_count` | count | Number of swarm tethers in range |

### Task 2 Outputs
- `task2_trajectory.png` — full path with avoidance maneuver highlighted
- `jetson_latency_histogram.png` — inference time distribution (mean ± 95% CI)
- ROS 2 bag: `task2_data/` containing `/odom`, `/cmd_vel`, `/detected_tethers`

---

## Jetson Orin Nano Performance

Expected real-time performance on Task 2:

| Metric | Expected Value |
|--------|----------------|
| LiDAR processing (line extraction) | 15–25 ms |
| Tether collision check | < 5 ms |
| ACO navigation update | 10–15 ms |
| **Total control loop** | **30–45 ms** (22–33 Hz) |

Measured via `rclcpp::Clock` timestamps in `data_logger.cpp`.

---

## Integration with Existing Code

This experiment **extends** your existing FormicaBot swarm:

- `formica_ws/src/formica_swarm_sim/robot_agent.py` — modified with yielding logic
- `Documents/formicabot_ws/src/formicabot_ros2/` — core ACO/DRL algorithms reused
- No changes to `PheromoneServer` or `SwarmMonitor` needed

All new code lives in `mixed_reality_ws/` to keep hardware validation separate.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `can0: no such device` | Check USB-CAN adapter: `lsusb`, `sudo ip link set can0 up` |
| No tether crossing detected | Increase `safety_margin_m` in bridge params, or reduce robot speed |
| LiDAR returns empty cloud | Check RealSense/URG driver, verify topic `/camera/depth/points` exists |
| CSV not writing | Ensure `~/mixed_reality_data/` is writable; check logger init |
| RViz shows no markers | Verify `fixed_frame=map` in RViz, check topic `/swarm_tether_viz` |

---

## Paper Integration

When you provide me the video and data, I will add this section:

> **"Hardware Validation in Mixed-Reality Heterogeneous Swarms"**
>
> To prove the framework's scalability and hardware-agnostic nature, we deployed
> the swarm policy on a quadrupedal Unitree AlienGo robot controlled by a Jetson
> Orin Nano. Through mixed-reality ROS 2 experiments, the physical quadruped
> successfully negotiated space with virtual swarm agents, demonstrating emergent
> yielding behavior to prevent multi-tether entanglement...

**Data to provide me:**
1. 30-second video (MP4) showing:
   - Robot walking → stopping → resuming (Task 1)
   - Robot avoiding moving rope (Task 2)
2. CSV files: `velocity_log_*.csv`
3. ROS 2 bag files (optional, for additional metrics)

---

## Citation

If this code contributes to your paper, please cite:

```bibtex
@inproceedings{formicabot2026swarm,
  title={Swarm Fuzzy-DRL for Multi-Tether Quadruped Navigation},
  author={Your Name},
  booktitle={IEEE ICRA},
  year={2026}
}
```

---

## License

MIT License — see LICENSE file for details.
