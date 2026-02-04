# Unified Navigation Framework for Tethered Robots

## Overview
A comprehensive framework designed for the unified navigation of tethered robotic systems. This project integrates motion control, mission planning, and simulation environments to facilitate research and development in tethered robotics.

## Project Structure
- **src/atlas_mission_control/**: Core ROS package for mission management.
  - **nodes/mission_manager_node.py**: Main node for coordinating robotic missions.
- **scripts/**: Utility scripts, including MATLAB implementations for analysis and visualization.
- **Dockerfile** & **docker-compose.yaml**: Containerization setup for consistent development and deployment.
- **setup.sh**: Environment initialization script.
- **requirements.txt**: Python dependencies.
- **result_analyzer.py**: Tools for processing and analyzing simulation data.
- **test_environment.py**: Automated testing suite.

## Installation
To set up the environment, run:
```bash
./setup.sh
```
Ensure all dependencies are installed:
```bash
pip install -r requirements.txt
```

## Usage
### Running in Docker
You can launch the full simulation environment using Docker Compose:
```bash
docker-compose up
```
Or use the provided convenience script:
```bash
./run_full_sim.sh
```

### Mission Control
The mission manager can be launched via ROS:
```bash
rosrun atlas_mission_control mission_manager_node.py
```

## Features
- Unified control architecture for tethered robots.
- Integrated simulation and analysis tools.
- Easy-to-use containerized setup.
- Support for autonomous missions.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.