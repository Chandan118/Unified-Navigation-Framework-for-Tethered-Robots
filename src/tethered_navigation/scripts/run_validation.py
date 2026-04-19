#!/usr/bin/env python3
"""
Quick Validation Runner - 快速验证实验
运行每种场景各1次，验证系统稳定性后再批量运行
"""

import subprocess
import sys
from pathlib import Path

SIMULATOR = Path('/home/jetson/ros2_ws/src/tethered_navigation/scripts/swarm_simulator_standalone.py')

# 验证矩阵：每种场景运行1次
VALIDATION_MATRIX = [
    ('bottleneck', 'baseline', 5),
    ('bottleneck', 'cooperative', 5),
    ('crossing', 'baseline', 6),
    ('crossing', 'cooperative', 6),
    ('expansion', 'baseline', 10),
    ('expansion', 'cooperative', 10),
]

def run_experiment(scenario, mode, num_robots, trial):
    cmd = [
        'python3', str(SIMULATOR),
        '--scenario', scenario,
        '--mode', mode,
        '--num-robots', str(num_robots),
        '--seed', str(10000 + trial)
    ]
    
    print(f"\n{'='*60}")
    print(f"  {scenario.upper()} | {mode.upper()} | {num_robots} robots (Trial {trial})")
    print(f"{'='*60}")
    
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    
    if result.returncode == 0:
        print(result.stdout[-500:] if len(result.stdout) > 500 else result.stdout)
        return True
    else:
        print(f"❌ FAILED: {result.stderr[-300:]}")
        return False

def main():
    print("="*70)
    print("  Swarm Robotics Validation Runner")
    print("  (Quick check before full batch)")
    print("="*70)
    
    results = []
    
    for idx, (scenario, mode, num_robots) in enumerate(VALIDATION_MATRIX, 1):
        success = run_experiment(scenario, mode, num_robots, idx)
        results.append((scenario, mode, num_robots, success))
    
    # 摘要
    print("\n" + "="*70)
    print("  VALIDATION SUMMARY")
    print("="*70)
    
    for scenario, mode, num_robots, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"  {status}  {scenario:12s} {mode:12s} ({num_robots} robots)")
    
    all_passed = all(success for _, _, _, success in results)
    
    if all_passed:
        print("\n✅ All validation experiments passed!")
        print("   You can now run the full batch of 80 experiments.")
        print("   Use: python3 /home/jetson/ros2_ws/src/tethered_navigation/scripts/run_parallel_experiments.py")
    else:
        print("\n⚠️  Some experiments failed. Check output above.")
    
    print("="*70)
    
    return 0 if all_passed else 1

if __name__ == '__main__':
    sys.exit(main())
