#!/usr/bin/env python3
"""
Data Analysis & Plotting for Mixed-Reality Swarm Experiments
============================================================
Generates publication-ready figures for the paper:
  - Velocity plot showing yielding (Task 1)
  - Trajectory plot (Task 2)
  - Jetson Orin Nano latency histogram (Task 2)
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
from scipy import stats

def plot_velocity_yielding(csv_path, output_path):
    """Plot robot velocity during Task 1 yielding event."""
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(df['timestamp_s'], df['linear_vel_mps'],
            label='Linear Velocity', linewidth=2, color='#1f77b4')
    ax.plot(df['timestamp_s'], df['angular_vel_radps'],
            label='Angular Velocity', linewidth=2, color='#ff7f0e', alpha=0.7)

    # Highlight yielding periods
    yielding_mask = df['yielding_state'] == 1  # YIELDING=1
    if yielding_mask.any():
        ax.axvspan(df.loc[yielding_mask, 'timestamp_s'].iloc[0],
                   df.loc[yielding_mask, 'timestamp_s'].iloc[-1],
                   alpha=0.3, color='red', label='Yielding Period')

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity (m/s, rad/s)', fontsize=12)
    ax.set_title('Unitree AlienGo Velocity During Mixed-Reality Yielding (Task 1)', fontsize=14)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    print(f"Saved velocity plot: {output_path}")
    plt.close()

def plot_trajectory(csv_path, output_path):
    """Plot robot trajectory from odometry during Task 2."""
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots(figsize=(8, 8))

    # Trajectory line colored by speed
    points = np.array([df['robot_x'], df['robot_y']]).T
    segments = np.zeros((len(points)-1, 2, 2))
    segments[:,0,0] = df['robot_x'][:-1]
    segments[:,0,1] = df['robot_y'][:-1]
    segments[:,1,0] = df['robot_x'][1:]
    segments[:,1,1] = df['robot_y'][1:]

    # Color by velocity
    from matplotlib.collections import LineCollection
    speeds = df['linear_vel_mps'][1:].values
    lc = LineCollection(segments, cmap='viridis', linewidth=2)
    lc.set_array(speeds)
    line = ax.add_collection(lc)

    # Mark start and goal
    ax.scatter(df['robot_x'].iloc[0], df['robot_y'].iloc[0],
               s=120, marker='^', color='green', label='Start', zorder=5)
    ax.scatter(df['robot_x'].iloc[-1], df['robot_y'].iloc[-1],
               s=120, marker='*', color='red', label='Goal', zorder=5)

    # Mark tether crossing point (detected in data)
    # (would be loaded from detection log)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Unitree AlienGo Trajectory with Dynamic Tether Avoidance (Task 2)', fontsize=14)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    plt.colorbar(line, ax=ax, label='Speed (m/s)')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    print(f"Saved trajectory plot: {output_path}")
    plt.close()

def plot_jetson_latency(csv_path, output_path):
    """Plot computation latency histogram on Jetson Orin Nano."""
    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots(figsize=(10, 4))

    # Assuming 'computation_ms' column exists
    if 'computation_ms' in df.columns:
        latencies = df['computation_ms'].dropna()
        ax.hist(latencies, bins=30, edgecolor='black', alpha=0.7, color='#2ca02c')

        # Stats line
        mean_lat = latencies.mean()
    ax.set_xlabel('Computation Time (ms)', fontsize=12)
    ax.set_ylabel('Frequency', fontsize=12)
    ax.set_title(f'Jetson Orin Nano Inference Latency (μ={mean_lat:.2f}ms)', fontsize=14)
    ax.axvline(mean_lat, color='red', linestyle='--', linewidth=2, label=f'Mean: {mean_lat:.2f}ms')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    print(f"Saved latency plot: {output_path}")
    plt.close()

def generate_all_plots(data_dir='mixed_reality_data', output_dir='paper_figures'):
    """Generate all paper figures from experimental data."""
    os.makedirs(output_dir, exist_ok=True)

    # Find latest CSV files
    csv_files = sorted(glob.glob(f"{data_dir}/velocity_log_*.csv"))
    if not csv_files:
        print(f"No CSV files found in {data_dir}")
        return

    latest_csv = csv_files[-1]
    print(f"Analyzing: {latest_csv}")

    # Generate plots
    plot_velocity_yielding(
        latest_csv,
        f"{output_dir}/task1_velocity_yielding.png")

    plot_trajectory(
        latest_csv,
        f"{output_dir}/task2_trajectory_avoidance.png")

    plot_jetson_latency(
        latest_csv,
        f"{output_dir}/task2_jetson_latency.png")

    print(f"\nAll figures saved to {output_dir}/")

if __name__ == '__main__':
    generate_all_plots()
