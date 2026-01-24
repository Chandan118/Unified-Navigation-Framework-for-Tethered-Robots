#!/usr/bin/env python3
"""
Result Analyzer for ATLAS-T Simulation
Parses simulation logs and generates a summary report with performance metrics
"""

import pandas as pd
import numpy as np
import os
import glob
import matplotlib.pyplot as plt
from datetime import datetime

def analyze_latest_results(results_dir):
    """Analyze the most recent CSV result file"""
    list_of_files = glob.glob(os.path.join(results_dir, '*.csv'))
    if not list_of_files:
        print("No result files found in", results_dir)
        return

    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Analyzing latest results from: {latest_file}")
    
    df = pd.read_csv(latest_file)
    
    if df.empty:
        print("Empty data file.")
        return

    # Calculate metrics
    total_time = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    
    # Path length
    dx = np.diff(df['pos_x'])
    dy = np.diff(df['pos_y'])
    path_length = np.sum(np.sqrt(dx**2 + dy**2))
    
    # Average tension
    avg_tension = df['tether_tension'].mean()
    max_tension = df['tether_tension'].max()
    
    # Average complexity
    avg_complexity = df['complexity_score'].mean()
    
    # State distribution
    state_counts = df['planner_state'].value_counts(normalize=True) * 100
    
    # Generate Report
    report_filename = latest_file.replace('.csv', '_report.txt')
    with open(report_filename, 'w') as f:
        f.write("==========================================\n")
        f.write("ATLAS-T SIMULATION PERFORMANCE REPORT\n")
        f.write("==========================================\n")
        f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Source Data: {os.path.basename(latest_file)}\n\n")
        
        f.write("CORE METRICS:\n")
        f.write(f"- Total Simulation Time: {total_time:.2f} seconds\n")
        f.write(f"- Total Distance Traveled: {path_length:.2f} meters\n")
        f.write(f"- Average Speed: {path_length/total_time:.2f} m/s\n")
        f.write(f"- Average Tether Tension: {avg_tension:.2f} N\n")
        f.write(f"- Maximum Tether Tension: {max_tension:.2f} N\n")
        f.write(f"- Average Scene Complexity: {avg_complexity:.2f}\n\n")
        
        f.write("NAVIGATION STATES (% OF TIME):\n")
        for state, percentage in state_counts.items():
            f.write(f"- {state}: {percentage:.1f}%\n")
        
        f.write("\n==========================================\n")
        f.write("ANALYSIS COMPLETE\n")
        
    print(f"Report generated: {report_filename}")
    
    # Optional: Generate a quick plot if display is available (will save to file instead)
    try:
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(df['timestamp'], df['tether_tension'], label='Tension (N)')
        plt.axhline(y=40.0, color='r', linestyle='--', label='Critical Threshold')
        plt.ylabel('Tension (N)')
        plt.legend()
        plt.title('Tension Over Time')
        
        plt.subplot(2, 1, 2)
        plt.plot(df['timestamp'], df['complexity_score'], label='Complexity', color='g')
        plt.ylabel('Complexity Score')
        plt.xlabel('Time (s)')
        plt.legend()
        
        plot_filename = latest_file.replace('.csv', '_plot.png')
        plt.savefig(plot_filename)
        print(f"Plot saved: {plot_filename}")
    except Exception as e:
        print(f"Could not generate plot: {e}")

if __name__ == '__main__':
    results_dir = '/home/chandan/atlas_ws/results'
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    analyze_latest_results(results_dir)
