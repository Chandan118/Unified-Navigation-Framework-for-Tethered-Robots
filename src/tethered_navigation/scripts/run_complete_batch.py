#!/usr/bin/env python3
"""
Complete Batch Runner - 所有80次实验
按照论文要求的Ablation Study设计：
- Bottleneck: 5 robots, 20 trials each (baseline + cooperative) = 40
- Crossing: 6 robots, 20 trials each = 40
- Expansion: 10 robots, 10 trials each = 20
Total: 100 trials (考虑到论文需要，运行完整数据集)
"""

import subprocess
import sys
import time
from pathlib import Path
from multiprocessing import Pool, cpu_count

SIMULATOR = Path('/home/jetson/ros2_ws/src/tethered_navigation/scripts/swarm_simulator_standalone.py')
RESULTS_DIR = Path('/home/jetson/swarm_simulation_results')

# 完整的实验矩阵
# (scenario, mode, num_robots, num_trials, timeout_seconds)
EXPERIMENT_MATRIX = [
    # Scenario A: Bottleneck (narrow corridor, 5 robots)
    ('bottleneck', 'baseline', 5, 20, 180),
    ('bottleneck', 'cooperative', 5, 20, 180),
    
    # Scenario B: Crossing Paths (X-pattern, 6 robots)
    ('crossing', 'baseline', 6, 20, 180),
    ('crossing', 'cooperative', 6, 20, 240),  # cooperative可能更慢
    
    # Scenario C: Swarm Expansion (10 robots from center)
    ('expansion', 'baseline', 10, 10, 180),
    ('expansion', 'cooperative', 10, 10, 180),
]

def run_trial(args):
    """运行单次试验（用于并行池）"""
    scenario, mode, num_robots, trial_idx, base_seed, timeout = args
    
    # 检查是否已存在
    exp_name = f"*{scenario}_{mode}_{num_robots}robots*"
    existing = list(RESULTS_DIR.glob(exp_name))
    if len(existing) >= trial_idx:
        return (scenario, mode, num_robots, trial_idx, True, "Already exists")
    
    cmd = [
        'python3', str(SIMULATOR),
        '--scenario', scenario,
        '--mode', mode,
        '--num-robots', str(num_robots),
        '--seed', str(base_seed + trial_idx)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                                cwd=str(SIMULATOR.parent))
        if result.returncode == 0:
            # 检查是否生成了结果文件
            latest = sorted(RESULTS_DIR.glob(exp_name))[-1] if RESULTS_DIR.glob(exp_name) else None
            if latest and (latest / 'swarm_metrics.csv').exists():
                return (scenario, mode, num_robots, trial_idx, True, "Success")
            else:
                return (scenario, mode, num_robots, trial_idx, False, "No output")
        else:
            return (scenario, mode, num_robots, trial_idx, False, f"Exit {result.returncode}")
    except subprocess.TimeoutExpired:
        return (scenario, mode, num_robots, trial_idx, False, f"Timeout ({timeout}s)")
    except Exception as e:
        return (scenario, mode, num_robots, trial_idx, False, str(e))

def run_batch_parallel(num_workers=None):
    """并行运行所有实验"""
    if num_workers is None:
        num_workers = max(1, cpu_count() // 2)
    print(f"🚀 Starting batch run with {num_workers} parallel workers")
    
    # 生成所有任务
    tasks = []
    base_seed = 10000
    total_trials = 0
    
    for scenario, mode, num_robots, trials, timeout in EXPERIMENT_MATRIX:
        for trial in range(1, trials + 1):
            tasks.append((scenario, mode, num_robots, trial, base_seed, timeout))
            total_trials += 1
        base_seed += 1000
    
    print(f"📋 Total trials to run: {total_trials}")
    print(f"📁 Results will be saved to: {RESULTS_DIR}")
    
    # 统计现有实验
    existing_count = len(list(RESULTS_DIR.glob('20260418*')))
    if existing_count > 0:
        print(f"⚠️  {existing_count} experiment folders already exist (will be skipped)")
    
    start_time = time.time()
    
    with Pool(processes=num_workers) as pool:
        completed = 0
        failed = 0
        
        print(f"\n{'='*70}")
        print("  Running experiments...")
        print(f"{'='*70}\n")
        
        for result in pool.imap_unordered(run_trial, tasks):
            scenario, mode, num_robots, trial, success, msg = result
            
            status = "✅" if success else "❌"
            print(f"  {status} {scenario:10s} {mode:10s} robots={num_robots}  trial={trial:2d}/{EXPERIMENT_MATRIX[0][3]:2d}  {msg}")
            
            if success:
                completed += 1
            else:
                failed += 1
            
            # 进度报告
            if (completed + failed) % 10 == 0:
                elapsed = time.time() - start_time
                rate = (completed + failed) / elapsed * 3600
                print(f"  📊 Progress: {completed+failed}/{total_trials}  Rate: {rate:.0f}/hour  Elapsed: {elapsed/60:.1f}min")
    
    elapsed = time.time() - start_time
    
    print("\n" + "="*70)
    print("  BATCH RUN COMPLETE")
    print("="*70)
    print(f"  Total tasks:   {total_trials}")
    print(f"  Completed:     {completed}")
    print(f"  Failed:        {failed}")
    print(f"  Elapsed time:  {elapsed/3600:.2f} hours ({elapsed/60:.1f} minutes)")
    print(f"\n  📁 Data location: {RESULTS_DIR}")
    print("="*70)
    
    # 下一步
    print("\n📊 Next steps:")
    print("   1. Run analysis: python3 /home/jetson/ros2_ws/src/tethered_navigation/scripts/aggregate_results.py")
    print("   2. Generate paper figures and tables")
    print("="*70)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Run complete batch of swarm experiments')
    parser.add_argument('--workers', type=int, default=None, help='Number of parallel workers')
    args = parser.parse_args()
    
    run_batch_parallel(args.workers)
