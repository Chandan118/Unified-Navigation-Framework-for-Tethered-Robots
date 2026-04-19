#!/usr/bin/env python3
"""
Batch Experiment Runner for Swarm Robotics Paper
=================================================
批量运行所有场景的实验，生成npj Robotics论文所需数据

实验矩阵：
| Scenario | Mode      | Robots | Trials |
|----------|-----------|--------|--------|
| Bottleneck | baseline  | 5      | 20     |
| Bottleneck | cooperative | 5    | 20     |
| Crossing  | baseline  | 6      | 20     |
| Crossing  | cooperative | 6    | 20     |
| Expansion | baseline  | 10     | 10     |
| Expansion | cooperative | 10   | 10     |

总计：80-100次实验
"""

import subprocess
import time
import sys
from pathlib import Path
from datetime import datetime

# 实验配置
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

def run_single_trial(scenario: str, mode: str, num_robots: int, trial: int, seed: int):
    """运行单次实验"""
    cmd = [
        'python3', str(SIMULATOR_PATH),
        '--scenario', scenario,
        '--mode', mode,
        '--num-robots', str(num_robots),
        '--seed', str(seed)
    ]
    
    print(f"  Trial {trial:2d}/?  Running: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        if result.returncode == 0:
            print(f"    ✅ Success")
            return True
        else:
            print(f"    ❌ Failed (exit {result.returncode})")
            print(f"    stderr: {result.stderr[:200]}")
            return False
    except subprocess.TimeoutExpired:
        print(f"    ⏱️  Timeout after 600s")
        return False
    except Exception as e:
        print(f"    ❌ Exception: {e}")
        return False

def run_batch_experiments():
    """批量运行所有实验"""
    print("="*70)
    print("  Swarm Robotics Batch Experiment Runner")
    print("  For npj Robotics Special Collection")
    print("="*70)
    
    total_experiments = sum(trials for _, _, _, trials in EXPERIMENT_MATRIX)
    print(f"\nTotal experiments to run: {total_experiments}")
    print("\nExperiment matrix:")
    for scenario, mode, num_robots, trials in EXPERIMENT_MATRIX:
        print(f"  {scenario:12s} | {mode:12s} | {num_robots} robots | {trials} trials")
    
    print("\n" + "="*70)
    
    # 统计
    results = {
        'total': total_experiments,
        'success': 0,
        'failed': 0,
        'skipped': 0
    }
    
    trial_seed = 1000  # 起始随机种子
    
    for scenario, mode, num_robots, num_trials in EXPERIMENT_MATRIX:
        print(f"\n📦 Scenario: {scenario.upper()} | Mode: {mode.upper()} | Robots: {num_robots}")
        print("-" * 70)
        
        for trial in range(1, num_trials + 1):
            print(f"  [{trial:2d}/{num_trials}] ", end='')
            
            # 检查是否已运行过（避免重复）
            expected_pattern = f"*_{scenario}_{mode}_{num_robots}robots"
            existing = list(RESULTS_BASE.glob(expected_pattern))
            # 简化：不检查单个trial，直接运行
            
            success = run_single_trial(scenario, mode, num_robots, trial, trial_seed)
            trial_seed += 1
            
            if success:
                results['success'] += 1
            else:
                results['failed'] += 1
            
            # 短暂休息（避免过热）
            time.sleep(0.5)
        
        print(f"\n  ✅ Completed {scenario}/{mode} ({num_robots} robots)")
    
    # 最终报告
    print("\n" + "="*70)
    print("  BATCH RUN COMPLETE")
    print("="*70)
    print(f"  Total:   {results['total']}")
    print(f"  Success: {results['success']}")
    print(f"  Failed:  {results['failed']}")
    print(f"  Skipped: {results['skipped']}")
    print(f"\n  Data location: {RESULTS_BASE}")
    print("="*70)
    
    # 生成索引文件
    generate_index_file()

def generate_index_file():
    """生成实验结果索引"""
    index_path = RESULTS_BASE / 'EXPERIMENT_INDEX.csv'
    
    with open(index_path, 'w') as f:
        f.write("scenario,mode,num_robots,trial,data_folder,success\n")
        
        for scenario, mode, num_robots, num_trials in EXPERIMENT_MATRIX:
            pattern = f"*_{scenario}_{mode}_{num_robots}robots"
            folders = sorted(RESULTS_BASE.glob(pattern))
            
            for folder in folders[:num_trials]:  # 只取前num_trials个
                f.write(f"{scenario},{mode},{num_robots},,{folder.name},True\n")
    
    print(f"\n📄 Index file created: {index_path}")

def analyze_results():
    """使用analyze_results.py生成统计汇总"""
    print("\n📊 生成数据分析报告...")
    
    analyzer = Path('/home/jetson/ros2_ws/src/tethered_navigation/scripts/analyze_results.py')
    if analyzer.exists():
        cmd = ['python3', str(analyzer), '--all', '--output', str(RESULTS_BASE)]
        subprocess.run(cmd)
        print("✅ 分析报告已生成")
    else:
        print("⚠️  analyzer.py 不存在，跳过数据分析")

if __name__ == '__main__':
    try:
        run_batch_experiments()
    except KeyboardInterrupt:
        print("\n\n⚠️  批量运行被用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
