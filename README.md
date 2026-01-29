# Unified Navigation Framework for Tethered Robots
## ROS 1 Noetic Implementation (Single & Swarm)

This project implements a comprehensive simulation of the Adaptive Hybrid Navigation Framework for Tethered Robots, extended for **Swarm Robotics** (up to 10 robots) for high-level coordination and performance analysis.

## Features

### High-Fidelity Simulation
- **Procedurally Generated Environments**: Perlin noise-based warehouse world generation
- **Physical Tether Model**: 40-link chain simulation with realistic physics and damping
- **Dynamic Obstacles**: Actor-based moving obstacles for realistic scenarios
- **Noisy Sensors**: LiDAR, IMU, and RGB-D camera with realistic noise models

### Hybrid Navigation System
1. **Sensor Fusion**: Extended Kalman Filter with dynamic weight adjustment (Eq. 8-9)
2. **Scene Recognition**: CNN-based complexity analysis
3. **Hybrid Planner**: Three-state FSM
   - Motion-to-Goal
   - Enhanced Bug Algorithm (wall following)
   - Tether Recovery
4. **Fuzzy Logic Controller**: 15+ rules for smooth navigation
5. **Genetic Algorithm Optimizer**: Automatic parameter tuning
6. **Tether Awareness**: Real-time tension calculation and snag detection

### Swarm Robotics Extension
- **Multi-Robot Coordination**: Centralized task allocation for 10 robots
- **Namespace Isolation**: Full topic/node namespacing for each robot (`/robot_1`, `/robot_2`, etc.)
- **Proximity Monitoring**: Real-time inter-robot spacing and collision warnings
- **Swarm Data Logging**: Unified telemetry collection for all robots into a single CSV
- **Aggregated Analytics**: Swarm-level metrics including workload balance and average tether stress

## Containerized Environment (Recommended)

To bypass dependency issues on modern hosts (e.g., Ubuntu 22.04), a Dockerized workflow is provided.

### Prerequisites
- Docker
- Docker Compose (v1.29+)

### Quick Start with Docker
```bash
cd atlas_ws
# Build and run the swarm simulation
docker-compose up --build
```

## System Requirements (Native)

- **OS**: Ubuntu 20.04
- **ROS**: ROS 1 Noetic
- **Python**: 3.8+
- **Gazebo**: 11.x

## Dependencies

### ROS Packages
```bash
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install ros-noetic-gazebo-ros-pkgs
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-velodyne-simulator
sudo apt-get install ros-noetic-hector-gazebo-plugins
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-joint-state-publisher
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-tf
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-rqt-plot
```

### Python Packages
```bash
pip3 install numpy scipy
pip3 install scikit-fuzzy
pip3 install opencv-python
pip3 install noise  # For Perlin noise generation
```

## Installation

### 1. Clone and Build
```bash
cd ~/Documents/Soft\ Computing\ Techniques/Journal\ of\ Field\ Robotics/manuscript/projecet/atlas_ws
catkin_make
source devel/setup.bash
```

### 2. Generate World (Optional)
```bash
rosrun atlas_gazebo world_generator.py
```

## Usage

### Swarm Simulation (10 Robots)
```bash
# Launch centralized coordinator and all navigation stacks
roslaunch hybrid_navigation master_swarm.launch
```

### Single Robot Simulation
```bash
# Launch complete single-robot system
roslaunch hybrid_navigation master.launch
```

### Analyzing Results
After running the simulation, data is saved in the `results/` folder. Use the analyzer to generate reports and plots:
```bash
python3 result_analyzer.py
```
*Note: The analyzer automatically detects if data is single-robot or multi-robot.*

## Monitoring and Visualization

### RViz
- Robot model with tether visualization
- LiDAR point cloud
- Color-coded tether (Green=low tension, Yellow=medium, Red=high/snagged)
- Navigation path

### RQT Plot
Monitor in real-time:
- `/tether_status/length` - Tether length
- `/tether_status/tension` - Tether tension
- `/scene_complexity/complexity_score` - Scene complexity

### Topics

**Subscribed:**
- `/scan` - LaserScan data
- `/odom` - Odometry
- `/imu/data` - IMU data
- `/camera/color/image_raw` - RGB camera
- `/move_base_simple/goal` - Navigation goal

**Published:**
- `/cmd_vel` - Velocity commands
- `/tether_status` - Tether state
- `/scene_complexity` - Scene analysis
- `/fused_pose` - Sensor-fused pose
- `/tether_visualization` - Tether marker
- `/planner_state` - Current FSM state

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                    │
│  ┌──────────┐  ┌──────────┐  ┌────────────────────┐   │
│  │  ATLAS-T │  │  Tether  │  │  Dynamic Obstacles │   │
│  │  Robot   │  │  Chain   │  │     (Actors)       │   │
│  └──────────┘  └──────────┘  └────────────────────┘   │
└─────────────────────────────────────────────────────────┘
           │                    │                  │
           ▼                    ▼                  ▼
    ┌──────────┐        ┌──────────────┐   ┌────────────┐
    │  Sensors │        │    Tether    │   │   Scene    │
    │  Fusion  │        │   Tension    │   │Recognition │
    │   (EKF)  │        │     Node     │   │   (CNN)    │
    └──────────┘        └──────────────┘   └────────────┘
           │                    │                  │
           └────────────────────┴──────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │  Hybrid Planner  │
                    │      (FSM)       │
                    └──────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ Fuzzy Controller │
                    │   (125 Rules)    │
                    └──────────────────┘
                              │
                              ▼
                         /cmd_vel
```

## Parameter Tuning

### Fuzzy Controller (`fuzzy_controller_node`)
- `linear_speed`: Base linear velocity (default: 0.5 m/s)
- `max_angular_speed`: Maximum angular velocity (default: 1.0 rad/s)
- `safety_distance`: Safety margin from obstacles (default: 1.0 m)

### Hybrid Planner (`hybrid_planner_node`)
- `obstacle_threshold`: Distance to trigger bug algorithm (default: 1.5 m)
- `wall_follow_distance`: Target distance for wall following (default: 0.7 m)
- `max_tether_usage`: Max tether usage ratio (default: 0.95)

### Tether Tension (`tether_tension_node`)
- `max_tether_length`: Maximum tether length (default: 30.0 m)
- `tension_coefficient`: Tension calculation coefficient (default: 1.5)

### GA Optimizer (`ga_optimizer_node`)
- `population_size`: GA population size (default: 10)
- `mutation_rate`: Mutation probability (default: 0.1)
- `optimization_interval`: Evolution interval in seconds (default: 120)

## Testing

### Manual Navigation Test
1. Launch the system
2. Use RViz "2D Nav Goal" tool to set goals
3. Observe tether visualization and state transitions
4. Monitor rqt_plot for metrics

### Autonomous Test
```bash
# Run predefined waypoint sequence
rosrun hybrid_navigation waypoint_test.py
```

## Expected Behavior

1. **Motion-to-Goal**: Robot moves directly toward goal when path is clear
2. **Enhanced Bug**: When obstacle detected, robot follows wall while heading toward goal
3. **Tether Recovery**: If tether approaches limit or snags, robot backs up toward base station
4. **Color Coding**: Tether changes color based on tension (green→yellow→orange→red)

## Troubleshooting

### Robot doesn't move
- Check `/cmd_vel` topic: `rostopic echo /cmd_vel`
- Verify Gazebo physics: Robot should not be stuck in ground
- Check tether: Excessive tension may prevent movement

### Sensors not publishing
- Verify Gazebo plugins are loaded
- Check `rostopic list` for expected topics
- Restart Gazebo if sensors freeze

### Build errors
- Ensure all dependencies are installed
- Run `rosdep install --from-paths src --ignore-src -r -y`
- Clean build: `catkin_make clean && catkin_make`

### Python import errors
- Check Python 3 compatibility
- Install missing packages: `pip3 install <package>`
- Verify node permissions: `chmod +x nodes/*.py`

## Manuscript and Analysis

The repository includes the latest research manuscript and MATLAB-based data analysis scripts:

- **Integrated Analysis**: The Python `result_analyzer.py` has been updated to calculate core metrics (Path Efficiency, Collision Rate, Entanglement Risk) based on the mathematical models provided in the MATLAB scripts.

## Advanced Performance Metrics

The analysis framework now includes metrics derived from publication standards:
1. **Path Efficiency (PLR %)**: Ratio of Euclidean distance to actual traveled path.
2. **Inferred Collision Rate**: Detection of tension spikes and abrupt velocity changes per meter.
3. **Entanglement Risk**: Evaluated based on recovery state frequency and high-tension duration.

## Citation

If you use this code, please cite the following paper:

```text
Chandan Sheikder, Unified Navigation Framework for Tethered Robots, [Year].
```

## License

MIT License - See LICENSE file for details

## Authors

Chandan Sheikder - chandan@bit.edu.cn

## Acknowledgments

- Based on research in Adaptive Hybrid Navigation for Tethered Robots
- ROS Community for excellent documentation
- Gazebo Simulator team