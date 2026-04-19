#!/usr/bin/env python3
"""
Mixed-Reality Swarm Experiment Runner
=====================================
Runs Task 1 (Virtual Robot Yielding) and Task 2 (Physical Tether Avoidance)
with the Unitree AlienGo on Jetson Orin Nano.

Usage:
  python3 run_mixed_reality_experiment.py --task 1
  python3 run_mixed_reality_experiment.py --task 2
"""

import argparse
import subprocess
import time
import sys
import os
import signal

def run_task(task_id, ros2_ws_path):
    """Run mixed reality experiment task."""
    print(f"\n{'='*60}")
    print(f"  Task {task_id}: Mixed-Reality Swarm Yielding Test")
    print(f"{'='*60}\n")

    if task_id == 1:
        print("Task 1: Virtual Robot Tether Crossing")
        print("  - Launch tether estimator (virtual robots)")
        print("  - Launch Unitree bridge (yielding on physical robot)")
        print("  - Record velocity data to CSV")
        print("  - Expected: Unitree stops when virtual tether crosses\n")
    elif task_id == 2:
        print("Task 2: Physical Tether Dynamic Avoidance")
        print("  - Attach physical rope to Aliengo")
        print("  - Use LiDAR to detect crossing rope")
        print("  - Record trajectory and Jetson latency\n")
    else:
        print("Invalid task ID")
        return

    # Source ROS 2
    source_cmd = f"source /opt/ros/humble/setup.bash && "
    if ros2_ws_path:
        source_cmd += f"source {ros2_ws_path}/install/setup.bash && "

    # Launch mixed reality experiment
    launch_cmd = (
        source_cmd +
        f"ros2 launch unitree_aliengo_bridge mixed_reality_experiment.launch.py"
    )

    print("Launch command:")
    print(f"  {launch_cmd}\n")
    print("Press Ctrl+C to stop recording and exit.\n")

    try:
        # Run the launch file
        proc = subprocess.Popen(
            launch_cmd,
            shell=True,
            executable='/bin/bash',
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        # Stream output
        while True:
            line = proc.stdout.readline()
            if not line and proc.poll() is not None:
                break
            if line:
                print(line.rstrip())
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\nExperiment interrupted. Shutting down...")
        proc.send_signal(signal.SIGINT)
        proc.wait()
    except Exception as e:
        print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(description='Run mixed-reality swarm experiments')
    parser.add_argument('--task', type=int, required=True, choices=[1,2],
                       help='Task to run: 1=virtual robot yielding, 2=physical tether avoidance')
    parser.add_argument('--ws', type=str, default='/home/chandan/mixed_reality_ws',
                       help='ROS 2 workspace path')

    args = parser.parse_args()
    run_task(args.task, args.ws)

if __name__ == '__main__':
    main()
