# Usage Guide

How to run simulations and experiments.

## 🚀 Running the Full Simulation

The easiest way to start is using the provided shell script or main launch file:

```bash
roslaunch hybrid_navigation master.launch
```

This will:
1.  Start `roscore`.
2.  Launch Gazebo with the warehouse world.
3.  Spawn the ATLAS robot.
4.  Start the navigation stack (planner, fuzzy controller, sensor fusion).
    *   *Note: Fuzzy controller includes a safety fallback to slow creep if rules are undefined but path is clear.*
5.  Open Rviz for visualization.
    *   *Note: Tether visualization now supports smoothed tension transitions.*

## 🧪 Running Individual Components

If you need to debug specific nodes:

**Launch Gazebo only:**
```bash
roslaunch atlas_gazebo spawn_robot.launch
```

**Start Navigation Logic:**
```bash
roslaunch hybrid_navigation hybrid_navigation.launch
```

## 📊 Visualization

**Rviz** is pre-configured to show:
-   Robot Model (TF tree)
-   Laser Scan data
-   Global Path (Green line)
-   Local Plan / Trajectory
-   Costmaps

To create a goal, use the **2D Nav Goal** tool in Rviz header.

## 📈 Analyzing Results

Use the result analyzer script to generate plots from run data:

```bash
python3 result_analyzer.py
```
*(Ensure you have `matplotlib` installed)*

---
[Return to Home](Home.md)
