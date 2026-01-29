#!/usr/bin/env python3
"""
Result Analyzer for ATLAS-T Simulation
Parses simulation logs and generates a summary report with performance metrics
Supports both single-robot and multi-robot (swarm) CSV formats.
"""

import pandas as pd
import numpy as np
import os
import glob
import matplotlib.pyplot as plt
from datetime import datetime


def analyze_latest_results(results_dir):
    """Analyze the most recent CSV result file"""
    list_of_files = glob.glob(os.path.join(results_dir, "*.csv"))
    if not list_of_files:
        print("No result files found in", results_dir)
        return

    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Analyzing latest results from: {latest_file}")

    df = pd.read_csv(latest_file)

    if df.empty:
        print("Empty data file.")
        return

    # Branch depending on whether we have multi-robot data
    if "robot_id" not in df.columns:
        analyze_single_robot(df, latest_file)
    else:
        analyze_multi_robot(df, latest_file)


def analyze_single_robot(df, latest_file):
    """Enhanced single-robot analysis including MATLAB-derived metrics."""
    # Calculate basic metrics
    total_time = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]

    # Path length
    dx = np.diff(df["pos_x"])
    dy = np.diff(df["pos_y"])
    path_length = np.sum(np.sqrt(dx ** 2 + dy ** 2))

    # Path Efficiency (PLR %)
    # Ratio of straight-line distance to actual path length
    start_pos = (df["pos_x"].iloc[0], df["pos_y"].iloc[0])
    end_pos = (df["pos_x"].iloc[-1], df["pos_y"].iloc[-1])
    euclidean_dist = np.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
    plr = (euclidean_dist / path_length * 100) if path_length > 0 else 0.0

    # Collision & Entanglement (Inferred from Tension and State)
    tension_threshold = 40.0
    collision_events = np.sum((df["tether_tension"] > tension_threshold) & (np.diff(df["tether_tension"], prepend=0) > 5.0))
    collision_rate = (collision_events / path_length) if path_length > 0 else 0.0
    
    recovery_time = len(df[df["planner_state"] == "TETHER_RECOVERY"]) * (total_time / len(df))
    entanglement_risk = (recovery_time / 3600.0) # hr

    # Average tension and complexity
    avg_tension = df["tether_tension"].mean()
    avg_complexity = df["complexity_score"].mean()

    # State distribution
    state_counts = df["planner_state"].value_counts(normalize=True) * 100

    # Generate Report
    report_filename = latest_file.replace(".csv", "_report.txt")
    with open(report_filename, "w") as f:
        f.write("==========================================\n")
        f.write("UNIFIED NAVIGATION FRAMEWORK ANALYSIS\n")
        f.write("==========================================\n")
        f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Source Data: {os.path.basename(latest_file)}\n\n")

        f.write("ADVANCED METRICS (Ref: MATLAB Supplemental):\n")
        f.write(f"- Path Efficiency (PLR): {plr:.2f}%\n")
        f.write(f"- Collision Rate: {collision_rate:.4f} events/m\n")
        f.write(f"- Entanglement Risk: {entanglement_risk:.4f} events/hr\n\n")

        f.write("CORE PERFORMANCE:\n")
        f.write(f"- Total Distance Traveled: {path_length:.2f} meters\n")
        f.write(f"- Average Speed: {path_length / total_time:.2f} m/s\n")
        f.write(f"- Average Tether Tension: {avg_tension:.2f} N\n")
        f.write(f"- Average Scene Complexity: {avg_complexity:.2f}\n\n")

        f.write("NAVIGATION STATES (% OF TIME):\n")
        for state, percentage in state_counts.items():
            f.write(f"- {state}: {percentage:.1f}%\n")

        f.write("\n==========================================\n")
        f.write("ANALYSIS COMPLETE\n")

    print(f"Enhanced report generated: {report_filename}")
    generate_professional_plots(df, latest_file)


def generate_professional_plots(df, latest_file):
    """Generate publication-quality plots (Nature/Scientific style)."""
    try:
        plt.style.use('seaborn-v0_8-whitegrid')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), dpi=300)
        
        # Panel A: Tether Dynamics
        ax1.plot(df["timestamp"], df["tether_tension"], color='#004B74', linewidth=1.5, label="Tension (N)")
        ax1.axhline(y=40.0, color='#D62728', linestyle='--', alpha=0.7, label="Critical Threshold")
        ax1.set_ylabel("Tether Tension (N)", fontweight='bold')
        ax1.set_title("Robot Navigation Performance Analysis", fontsize=12, fontweight='bold')
        ax1.legend(loc='upper right', frameon=True)
        
        # Panel B: Complexity vs State
        ax2.plot(df["timestamp"], df["complexity_score"], color='#2CA02C', linewidth=1.5, label="Complexity")
        ax2.set_ylabel("Complexity Score", fontweight='bold')
        ax2.set_xlabel("Time (s)", fontweight='bold')
        
        # Overlay state regions
        states = df["planner_state"].unique()
        colors = plt.cm.get_cmap('Pastel1', len(states))
        for i, state in enumerate(states):
            mask = df["planner_state"] == state
            ax2.fill_between(df["timestamp"], 0, 1, where=mask, color=colors(i), alpha=0.3, label=state)
        
        ax2.legend(loc='upper right', frameon=True)
        
        plt.tight_layout()
        plot_filename = latest_file.replace(".csv", "_professional_plot.png")
        plt.savefig(plot_filename)
        print(f"Professional plot saved: {plot_filename}")
    except Exception as e:
        print(f"Could not generate professional plot: {e}")


def analyze_multi_robot(df, latest_file):
    """Enhanced swarm analysis."""
    robots = sorted(df["robot_id"].unique())
    report_filename = latest_file.replace(".csv", "_report.txt")
    
    with open(report_filename, "w") as f:
        f.write("==========================================\n")
        f.write("UNIFIED SWARM PERFORMANCE REPORT\n")
        f.write("==========================================\n")
        f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        total_plr = []
        for robot_id in robots:
            sub = df[df["robot_id"] == robot_id].sort_values("timestamp")
            if sub.empty: continue
            
            # Re-use logic for path efficiency
            path_len = np.sum(np.sqrt(np.diff(sub["pos_x"])**2 + np.diff(sub["pos_y"])**2))
            euclidean = np.sqrt((sub["pos_x"].iloc[-1] - sub["pos_x"].iloc[0])**2 + (sub["pos_y"].iloc[-1] - sub["pos_y"].iloc[0])**2)
            plr = (euclidean / path_len * 100) if path_len > 0 else 0
            total_plr.append(plr)
            
            f.write(f"Robot {robot_id}: PLR={plr:.1f}%, Avg Tension={sub['tether_tension'].mean():.1f}N\n")

        f.write(f"\nSwarm Mean Path Efficiency: {np.mean(total_plr):.2f}%\n")
        f.write(f"Swarm Workload Balance (Std Dev): {np.std(total_plr):.2f}%\n")
        f.write("\n==========================================\n")

    print(f"Swarm report generated: {report_filename}")


if __name__ == "__main__":
    # Use current workspace results directory
    results_dir = "/Users/chandansheikder/Documents/Soft Computing Techniques/Robotics and Autonomous Systems/Manuscript/Npj/atlas_ws/results"
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    analyze_latest_results(results_dir)
