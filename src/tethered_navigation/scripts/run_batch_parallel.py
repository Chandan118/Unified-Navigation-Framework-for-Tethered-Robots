#!/usr/bin/env python3
"""
Batch Experiment Runner with Adaptive Timeout
==============================================
运行所有80次实验，根据场景自动调整超时时间
"""

import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime
from multiprocessing import Pool, cpu_count
import os

# 实验矩阵: (scenario, mode, num_robots, num_trials, timeout_seconds)
EXPERIMENT_MATRIX = [
    ('bottleneck', 'baseline', 5, 3, 300),     # 验证用3次
    ('bottleneck', 'cooperative', 5, 3, 300),  # 验证用3次
    ('crossing', 'baseline', 6, 3, 300),
    ('crossing', 'cooperative', 6, 3, 600),    # cooperative可能需要更长时间
    ('expansion', 'baseline', 10, 3, 300),
    ('expansion', 'cooperative', 10, 3, 300),
]

# 扩展：完整实验（用于最终数据）
FULL_EXPERIMENT_MATRIX = [
    ('bottleneck', 'baseline', 5, 20, 300),
    ('bottleneck', 'cooperative', 5, 20, 300),
    ('crossing', 'baseline', 6, 20, 300),
    ('crossing', 'cooperative', 6, 20, 600),
    ('expansion', 'baseline', 10, 10, 300),
    ('expansion', 'cooperative', 10, 10, 300),
]

SIMULATOR_PATH = Path('/home/jetson/ros2_ws/src/tethered_navigation/scripts/swarm_simulator_standalone.py')
RESULTS_BASE = Path('/home/jetson/swarm_simulation_results')

def run_trial_wrapper(args):
    """包装函数，用于multiprocessing.Pool"""
    scenario, mode, num_robots, trial, seed, timeout = args
    
    # 检查是否已存在
    pattern = f"*_{scenario}_{mode}_{num_robots}robots"
    existing = list(RESULTS_BASE.glob(pattern))
    if len(existing) >= trial:
        return (scenario, mode, num_robots, trial, True, "Already exists")
    
    cmd = [
        'python3', str(SIMULATOR_PATH),
        '--scenario', scenario,
        '--mode', mode,
        '--num-robots', str(num_robots),
        '--seed', str(seed + trial)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                                cwd=str(SIMULATOR_PATH.parent))
        if result.returncode == 0:
            return (scenario, mode, num_robots, trial, True, "Success")
        else:
            return (scenario, mode, num_robots, trial, False, f"Exit {result.returncode}")
    except subprocess.TimeoutExpired:
        return (scenario, mode, num_robots, trial, False, f"Timeout ({timeout}s)")
    except Exception as e:
        return (scenario, mode, num_robots, trial, False, str(e))

def run_batch(mode='validation', num_workers=None):
    """运行批量实验
    
    Args:
        mode: 'validation' (快速验证) 或 'full' (完整80次)
        num_workers: 并行进程数（默认使用一半CPU核心）
    """
    if mode == 'validation':
        matrix = EXPERIMENT_MATRIX
        print("🔬 Running VALIDATION batch (6 experiments, 3 trials each)")
    else:
        matrix = FULL_EXPERIMENT_MATRIX
        print("🚀 Running FULL batch (80 experiments)")
    
    total_trials = sum(trials for _, _, _, trials, _ in matrix)
    print(f"Total trials: {total_trials}")
    
    # 生成所有任务
    tasks = []
    base_seed = 10000
    
    for scenario, mode_str, num_robots, trials, timeout in matrix:
        for trial in range(1, trials + 1):
            tasks.append((scenario, mode_str, num_robots, trial, base_seed, timeout))
        base_seed += 1000
    
    print(f"Tasks to run: {len(tasks)}")
    
    if num_workers is None:
        num_workers = max(1, cpu_count() // 2)
    print(f"Using {num_workers} parallel workers\n")
    
    start_time = time.time()
    completed = 0
    failed = 0
    
    with Pool(processes=num_workers) as pool:
        for result in pool.imap_unordered(run_trial_wrapper, tasks):
            scenario, mode_str, num_robots, trial, success, msg = result
            
            status = "✅" if success else "❌"
            print(f"  {status} {scenario:12s} {mode_str:12s} robots={num_robots} trial={trial:2d}  {msg}")
            
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
    print(f"  Elapsed time:  {elapsed/3600:.2f} hours")
    print(f"\n  Data location: {RESULTS_BASE}")
    print("="*70)
    
    # 提示运行数据分析
    print("\n📊 Next step: Run analysis")
    print("   python3 /home/jetson/ros2_ws/src/tethered_navigation/scripts/aggregate_results.py")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Batch swarm experiment runner')
    parser.add_argument('--mode', choices=['validation', 'full'], default='validation',
                       help='Run validation (6 exp) or full batch (80 exp)')
    parser.add_argument('--workers', type=int, default=None,
                       help='Number of parallel workers')
    
    args = parser.parse_args()
    run_batch(args.mode, args.workers)

if __name__ == '__main__':
    main()
