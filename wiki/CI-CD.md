# CI/CD Pipelines

This repository uses **GitHub Actions** to ensure code quality and build integrity across multiple environments.

## 🛠️ Workflows

### 1. Python Package (`python-package.yml`)
*   **Triggers**: Push/PR to `main`, `master`, `ros2-development`, `ros1-noetic`.
*   **Purpose**: lints Python code with `flake8` and runs unit tests with `pytest`.
*   **Environment**: Ubuntu 20.04 / Ubuntu Latest (branch dependent).
*   **Python Versions**: 3.8, 3.9, 3.10.

### 2. ROS CI & Publish (`ros_ci.yml`)
*   **Triggers**: Push/PR to `main`, `master`, `ros2-development`, `ros1-noetic`.
*   **Purpose**: Builds the ROS workspace using `catkin_make` (ROS 1) or `colcon build` (ROS 2) and runs package tests.
*   **Environment**:
    *   **ROS 1**: `ubuntu-latest` (Dockerized `ros:noetic-ros-base`)
    *   **ROS 2**: `ubuntu-22.04` (Native `ros-tooling/setup-ros` setup)

## ✅ Branch Specifics

| Branch | ROS Version | OS | CI Status |
| :--- | :--- | :--- | :--- |
| `master` | ROS 2 Humble | Ubuntu 22.04 | Active |
| `ros1-noetic` | ROS 1 Noetic | Ubuntu 20.04 (via Docker) | Active |
| `ros2-development` | ROS 2 Humble | Ubuntu 22.04 | Active |
| `fix/flake8...` | ROS 2 Humble | Ubuntu 22.04 | Active/Merged |

## 🧪 How to Run Locally

To replicate the CI checks locally:

**Linting:**
```bash
flake8 . --count --select=E9,F63,F7,F82 --show-source --statistics --exclude=build,devel,install
```

**Testing:**
```bash
pytest
```
*(Make sure you are in the root directory)*

---
[Return to Home](Home.md)
