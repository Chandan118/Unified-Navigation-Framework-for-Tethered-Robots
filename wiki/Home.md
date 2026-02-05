# Welcome to the Unified Navigation Framework for Tethered Robots Wiki

This wiki serves as the central documentation hub for the **Unified Navigation Framework for Tethered Robots**. This project provides a robust simulation and control environment for tethered reliability in autonomous navigation.

## 🚀 Project Overview

The Unified Navigation Framework integrates:
-   **Hybrid Navigation Planning**: Combining global path planning with local fuzzy logic control.
-   **Tether Management**: Simulating tension and physical constraints of a tethered robot.
-   **ROS 1 & ROS 2 Support**: Designed for ROS Noetic (Ubuntu 20.04) with ongoing migration support for ROS 2 Humble.
-   **Gazebo Simulation**: High-fidelity simulation environments for testing and validation.

## 📊 Current Status

| Feature | Status | Notes |
| :--- | :--- | :--- |
| **CI/CD Pipeline** | 🟢 **Stable** | Passing on all branches (Noetic & Humble) |
| **Simulation** | 🟡 **Beta** | `atlas_gazebo` functioning, further tuning required |
| **Navigation** | 🟢 **Stable** | Hybrid A* + Fuzzy Logic operational |
| **Tether Physics** | 🟡 **Beta** | Basic tension model implemented |

## 🗺️ Roadmap

-   [x] **Phase 1**: Core Simulation & Basic Navigation (Completed)
-   [x] **Phase 2**: CI/CD Integration & Automated Testing (Completed)
-   [ ] **Phase 3**: Advanced Tether Dynamics & 3D Environment Support
-   [ ] **Phase 4**: Real-world Hardware Integration (ATLAS Platform)
-   [ ] **Phase 5**: Multi-robot Swarm Coordination

## 📚 Navigation

*   [**Installation Guide**](Installation.md): Get started with setting up the environment.
*   [**Usage Guide**](Usage.md): How to launch simulations and run experiments.
*   [**System Architecture**](Architecture.md): Deep dive into the packages and nodes.
*   [**CI/CD Workflows**](CI-CD.md): Details on our automated testing pipeline.
