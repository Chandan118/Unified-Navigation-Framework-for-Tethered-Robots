# System Architecture

The framework is modular, consisting of several ROS packages that handle specific aspects of the simulation and control.

## 📦 Core Packages

### 1. `hybrid_navigation`
The brain of the system. Implements the unified navigation logic.
*   **Nodes**:
    *   `hybrid_planner_node.py`: Integrates global path planning with local reactive control.
    *   `fuzzy_controller_node.py`: Implements fuzzy logic for obstacle avoidance and tether tension management.
    *   `ga_optimizer_node.py`: Genetic Algorithm for path optimization.
    *   `sensor_fusion_node.py`: Fuses odometry, IMU, and LiDAR data.
*   **Key Algorithms**: A*, Fuzzy Logic Control, Genetic Algorithms.

### 2. `atlas_description`
Contains the physical description of the robot.
*   **Content**: URDF/Xacro files defining the robot's geometry, joints, and physical properties.
*   **Sensors**: Defines mounting points for LiDAR, Camera, and IMU.

### 3. `atlas_gazebo`
Simulation environment configuration.
*   **Worlds**: Custom Gazebo worlds (e.g., `warehouse_world.world`) with obstacles as defined in the manuscript.
*   **Launch Files**: Scripts to spawn the robot and load the environment.

### 4. `atlas_mission_control`
High-level state machine for mission management.
*   **Nodes**:
    *   `mission_manager_node.py`: Handles state transitions (Idle, Navigating, ObstacleAvoidance, EmergencyStop).

## 🔄 Data Flow

1.  **Sensors (Gazebo)** publish `LaserScan`, `Imu`, and `Odom`.
2.  **`sensor_fusion_node`** aggregates this data into a coherent state estimate.
3.  **`hybrid_planner_node`** generates a global path to the goal.
4.  **`fuzzy_controller_node`** adjusts the velocity commands (`cmd_vel`) to follow the path while avoiding obstacles and maintaining safe tether tension.
5.  **`atlas_gazebo`** receives `cmd_vel` and updates the physics simulation.

---
[Return to Home](Home.md)
