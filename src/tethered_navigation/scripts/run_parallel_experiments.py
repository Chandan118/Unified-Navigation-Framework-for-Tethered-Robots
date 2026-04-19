#!/usr/bin/env python3
"""
Parallel Batch Experiment Runner for Swarm Robotics
====================================================
并行运行所有实验以加速数据收集

使用多进程并行运行多个仿真器
每个仿真器独立运行，互不干扰
"""

import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime
from multiprocessing import Pool, cpu_count
import os

# 实验矩阵
EXPERIMENT_MATRIX = [
    # (scenario, mode, num_robots, num_trials)
    ('bottleneck', 'baseline', 5, 20),
    ('bottleneck', 'cooperative', 5, 20),
    ('crossing', 'baseline', 6, 20),
    ('crossing', 'cooperative', 6, 20),
    ('expansion', 'baseline', 10, 10),
    ('expansion', 'cooperative', 10, 10),
]

SIMULATOR_PATH = Path('/home/jetson/ros2_ws/src/tethered_navigation/scripts/swarm_simulator_standalone.py')
RESULTS_BASE = Path('/home/jetson/swarm_simulation_results')

def run_trial_wrapper(args):
    """包装函数，用于multiprocessing.Pool"""
    scenario, mode, num_robots, trial, seed = args
    
    # 检查是否已存在
    pattern = f"*_{scenario}_{mode}_{num_robots}robots"
    existing = list(RESULTS_BASE.glob(pattern))
    if len(existing) >= trial:
        # 已存在，跳过
        return (scenario, mode, num_robots, trial, True, "Already exists")
    
    cmd = [
        'python3', str(SIMULATOR_PATH),
        '--scenario', scenario,
        '--mode', mode,
        '--num-robots', str(num_robots),
        '--seed', str(seed + trial)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600, cwd=str(SIMULATOR_PATH.parent))
        if result.returncode == 0:
            return (scenario, mode, num_robots, trial, True, "Success")
        else:
            return (scenario, mode, num_robots, trial, False, f"Exit {result.returncode}")
    except subprocess.TimeoutExpired:
        return (scenario, mode, num_robots, trial, False, "Timeout")
    except Exception as e:
        return (scenario, mode, num_robots, trial, False, str(e))

def main():
    print("="*70)
    print("  Parallel Swarm Experiment Runner")
    print("="*70)
    
    # 计算总实验数
    total_trials = sum(trials for _, _, _, trials in EXPERIMENT_MATRIX)
    print(f"Total trials: {total_trials}")
    
    # 生成所有任务
    tasks = []
    base_seed = 10000
    
    for scenario, mode, num_robots, trials in EXPERIMENT_MATRIX:
        for trial in range(1, trials + 1):
            tasks.append((scenario, mode, num_robots, trial, base_seed))
        base_seed += 1000
    
    print(f"Tasks to run: {len(tasks)}")
    print(f"Already completed: skip (checked via pattern matching)")
    
    # 使用多进程并行运行（使用一半CPU核心）
    num_workers = max(1, cpu_count() // 2)
    print(f"\nUsing {num_workers} parallel workers")
    print("\nStarting experiments...\n")
    
    start_time = time.time()
    completed = 0
    failed = 0
    
    with Pool(processes=num_workers) as pool:
        for result in pool.imap_unordered(run_trial_wrapper, tasks):
            scenario, mode, num_robots, trial, success, msg = result
            
            status = "✅" if success else "❌"
            print(f"  {status} {scenario:12s} {mode:12s} robots={num_robots} trial={trial:2d}  {msg}")
            
            if success:
                completed += 1
            else:
                failed += 1
    
    elapsed = time.time() - start_time
    
    print("\n" + "="*70)
    print("  BATCH RUN COMPLETE")
    print("="*70)
    print(f"  Total tasks:   {len(tasks)}")
    print(f"  Completed:     {completed}")
    print(f"  Failed:        {failed}")
    print(f"  Elapsed time:  {elapsed/3600:.1f} hours")
    print(f"\n  Data location: {RESULTS_BASE}")
    print("="*70)

if __name__ == '__main__':
    main()
