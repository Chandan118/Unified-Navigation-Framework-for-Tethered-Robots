# Adaptive Hybrid Navigation Framework for Tethered Robots
## ROS 1 Noetic Implementation

This project implements a comprehensive simulation of the Adaptive Hybrid Navigation Framework for Tethered Robots as described in the research paper, using ROS 1 Noetic and Gazebo.

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

## System Requirements

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

### Quick Start - Full System
```bash
# Terminal 1: Launch complete system
roslaunch hybrid_navigation master.launch
```

### Step-by-Step Launch

```bash
# Terminal 1: Start Gazebo with robot
roslaunch atlas_gazebo spawn_robot.launch

# Terminal 2: Start navigation stack
roslaunch hybrid_navigation hybrid_navigation.launch

# Terminal 3: Send navigation goal
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'odom'
pose:
  position:
    x: 10.0
    y: 10.0
    z: 0.0
  orientation:
    w: 1.0"
```

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

## Performance Metrics

The system can generate the following metrics for analysis:

1. **Path Efficiency**: Total path length / Optimal path length
2. **Tether Stress**: Average and max tension over mission
3. **Obstacle Avoidance**: Number of collisions avoided
4. **State Transitions**: Frequency of FSM state changes
5. **Parameter Convergence**: GA fitness over generations

## Citation

If you use this code, please cite the original research paper:

```
[Paper Citation Here]
```

## License

MIT License - See LICENSE file for details

## Authors

Chandan Sheikder - chandan@bit.edu.cn

## Acknowledgments

- Based on research in Adaptive Hybrid Navigation for Tethered Robots
- ROS Community for excellent documentation
- Gazebo Simulator team
