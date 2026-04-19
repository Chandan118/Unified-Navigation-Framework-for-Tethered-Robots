# Unified Navigation Framework for Tethered Robots
## ROS 2 Navigation Stack with Mixed Reality Validation

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18522121.svg)](https://doi.org/10.5281/zenodo.18522121)

This project implements a comprehensive **Cooperative Swarm Navigation Framework for Tethered Robots**, featuring ROS 2 navigation with real-world hardware validation and mixed reality experiments.

## Key Features

### Cooperative Swarm Navigation
- **Centralized Task Allocation**: Multi-robot coordination with up to 10 robots
- **Cooperative Scenarios**: Bottleneck, crossing paths, and expansion area navigation
- **Yielding Decision Protocol**: Robots dynamically yield based on mission priority and tether tension
- **Namespace Isolation**: Full topic/node namespacing for each robot (`/robot_1`, `/robot_2`, etc.)
- **Aggregated Analytics**: Swarm-level metrics including workload balance and average tether stress

### High-Fidelity Simulation
- **Procedurally Generated Environments**: SDF-based worlds for swarm scenarios
- **Physical Tether Model**: 40-link chain simulation with realistic physics
- **Dynamic Obstacles**: Actor-based moving obstacles for realistic scenarios
- **Noisy Sensors**: LiDAR, IMU, and RGB-D camera with realistic noise models

### Hybrid Navigation System
1. **Sensor Fusion**: Extended Kalman Filter with dynamic weight adjustment
2. **Scene Recognition**: CNN-based complexity analysis
3. **Hybrid Planner**: Three-state FSM
   - Motion-to-Goal
   - Enhanced Bug Algorithm (wall following)
   - Tether Recovery
4. **Fuzzy Logic Controller**: 15+ rules for smooth navigation
5. **Genetic Algorithm Optimizer**: Automatic parameter tuning
6. **Tether Awareness**: Real-time tension calculation and snag detection

### Mixed Reality Validation
- **Hardware-in-the-Loop**: Jetson Nano-based real robot experiments
- **Protocol Validation**: Task 1 (velocity tracking) and Task 2 (trajectory following)
- **Performance Metrics**: Real-world latency, accuracy, and stability measurements

## Repository Structure

```
Unified-Navigation-Framework-for-Tethered-Robots/
├── src/tethered_navigation/      # Main ROS 2 navigation package
│   ├── config/                   # EKF, laser merger, Nav2 params
│   ├── launch/                   # Launch files for scenarios
│   ├── msg/                      # Custom message definitions
│   ├── scripts/                  # Python nodes and analyzers
│   └── worlds/                   # SDF world files
├── mixed_reality_ws/             # Hardware validation workspace
│   ├── src/                      # Hardware interface nodes
│   ├── log/                      # Experiment logs
│   └── analyze_data.py           # Hardware data analyzer
├── results/                      # Simulation results and analysis
│   ├── aggregated_results/       # Paper-ready statistics
│   └── quick_analysis/           # Summary plots and CSV
├── paper_submission/             # Figures and data for publication
└── assets/figure_data/           # MATLAB figure generation scripts

```

## System Requirements

### ROS 2 Navigation Stack
- **OS**: Ubuntu 22.04 (Jammy)
- **ROS**: ROS 2 Humble
- **Python**: 3.10+
- **Gazebo**: Gazebo Harmonic (Ignition)
- **Navigation2**: Nav2 stack with EKF localization

### Hardware (for mixed reality)
- NVIDIA Jetson Nano (or equivalent)
- RP LiDAR A1/A2
- USB Camera
- Tethered mobile robot platform

## Installation

### 1. ROS 2 Workspace Setup
```bash
mkdir -p ~/tethered_ws/src
cd ~/tethered_ws
colcon build
source install/setup.bash
```

### 2. Build the Navigation Package
```bash
cd ~/tethered_ws/src/tethered_navigation
rosdep install --from-paths . --ignore-src -r -y
cd ~/tethered_ws
colcon build --packages-select tethered_navigation
source install/setup.bash
```

### 3. Install Python Dependencies
```bash
pip3 install numpy scipy opencv-python pandas matplotlib
```

## Usage

### Swarm Simulation Experiments

#### Cooperative Bottleneck Scenario
```bash
# Cooperative navigation (robots yield to each other)
ros2 launch tethered_navigation scenario_bottleneck_cooperative.launch.py

# Baseline (no cooperation)
ros2 launch tethered_navigation scenario_bottleneck_baseline.launch.py
```

#### Crossing Paths Scenario
```bash
ros2 launch tethered_navigation scenario_crossing_cooperative.launch.py
```

#### Expansion Area Scenario
```bash
ros2 launch tethered_navigation scenario_expansion_cooperative.launch.py
```

### Analyzing Results
```bash
# Analyze aggregated results
python3 scripts/analyze_results.py

# Generate quick summary
python3 scripts/quick_analysis.py

# Aggregate multiple experiment runs
python3 scripts/aggregate_results.py
```

### Mixed Reality Validation

#### Task 1: Velocity Tracking
```bash
./mixed_reality_ws/run_task1.sh
```

#### Task 2: Trajectory Following
```bash
./mixed_reality_ws/run_task2.sh
```

#### Analyze Hardware Data
```bash
python3 mixed_reality_ws/analyze_data.py
```

## Performance Metrics

### Simulation Metrics
- **Path Efficiency (PLR %)**: Ratio of Euclidean distance to actual traveled path
- **Inferred Collision Rate**: Detection of tension spikes per meter
- **Entanglement Risk**: Recovery state frequency and high-tension duration
- **Cooperation Rate**: Percentage of successful yielding events

### Hardware Metrics
- **Tracking Error**: Mean and max deviation from reference trajectory
- **Jetson Latency**: Real-time processing delay
- **Velocity Accuracy**: Command vs. actual velocity correlation

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation / Hardware             │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────────────┐   │
│  │  ATLAS-T │  │  Tether  │  │  Swarm Coordination      │   │
│  │  Robots  │  │  Chain   │  │  (Cooperative Protocol)   │   │
│  └──────────┘  └──────────┘  └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
           │                    │                       │
           ▼                    ▼                       ▼
    ┌──────────┐        ┌──────────────┐        ┌─────────────┐
    │  Sensors │        │    Tether    │        │  Swarm      │
    │  Fusion  │        │   Tension    │        │  Coordinator│
    │   (EKF)  │        │     Node     │        │             │
    └──────────┘        └──────────────┘        └─────────────┘
           │                    │                       │
           └────────────────────┴───────────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │  Hybrid Planner  │
                    │    (FSM + Nav2)  │
                    └──────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ Fuzzy Controller │
                    │   + Yielding     │
                    └──────────────────┘
                              │
                              ▼
                         /cmd_vel
```

## Scenario Descriptions

### Bottleneck Scenario
Robots navigate through a narrow passage. Cooperative mode enables robots to yield based on mission priority, reducing congestion and improving overall throughput.

### Crossing Paths Scenario
Multiple robots traverse intersecting paths. The cooperative protocol coordinates robot timing to prevent collisions.

### Expansion Area Scenario
Robots move from a confined area into an open space. Coordination ensures orderly expansion without collision.

## Parameter Tuning

### Swarm Coordinator (`swarm_coordinator.py`)
- `cooperation_enabled`: Enable/disable yielding protocol
- `yield_threshold`: Tension level to trigger yielding
- `mission_priority_offset`: Priority weight for mission urgency

### Navigation Stack (`nav2_params.yaml`)
- `controller_frequency`: Control loop rate (default: 20 Hz)
- `planner_frequency`: Path planning rate (default: 1 Hz)
- `recovery_timeout`: Time before recovery actions (default: 5 s)

### Tether Tension (`tether_state_broadcaster.py`)
- `max_tether_length`: Maximum tether length (default: 10.0 m)
- `tension_coefficient`: Tension calculation coefficient (default: 1.5)
- `yielding_tension_threshold`: Tension to trigger robot yielding

## Citation

If you use this code, please cite the following paper:

```text
Chandan Sheikder, Unified Navigation Framework for Tethered Robots, 2026.
DOI: 10.5281/zenodo.18522121
```

## License

MIT License - See LICENSE file for details

## Authors

Chandan Sheikder - chandan@bit.edu.cn

## Acknowledgments

- Based on research in Adaptive Hybrid Navigation for Tethered Robots
- ROS 2 Community for excellent documentation
- Nav2 Stack contributors
- Gazebo Simulator team
