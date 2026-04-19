#!/usr/bin/env python3
"""
Quick Final Validation - 所有场景快速测试
验证每个场景在两种模式下都能在合理时间内完成
"""

import subprocess
import sys
from pathlib import Path

SIMULATOR = Path('/home/jetson/ros2_ws/src/tethered_navigation/scripts/swarm_simulator_standalone.py')

# 测试矩阵：每个场景各1次
TEST_MATRIX = [
    ('bottleneck', 'baseline', 5),
    ('bottleneck', 'cooperative', 5),
    ('crossing', 'baseline', 6),
    ('crossing', 'cooperative', 6),
    ('expansion', 'baseline', 10),
    ('expansion', 'cooperative', 10),
]

# 超时时间（秒）
TIMEOUTS = {
    'bottleneck': 120,
    'crossing': 240,  # crossing可能需要更长时间
    'expansion': 180,
}

def run_test(scenario, mode, num_robots, trial):
    """运行单次测试"""
    timeout = TIMEOUTS.get(scenario, 180)
    
    cmd = [
        'python3', str(SIMULATOR),
        '--scenario', scenario,
        '--mode', mode,
        '--num-robots', str(num_robots),
        '--seed', str(90000 + trial)
    ]
    
    print(f"\n{'='*60}")
    print(f"  {scenario.upper()} | {mode.upper()} | {num_robots} robots (trial {trial})")
    print(f"  Timeout: {timeout}s")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        
        if result.returncode == 0:
            # 解析输出找到结果摘要
            output = result.stdout
            # 查找summary部分
            if "All reached goal:" in output:
                lines = output.split('\n')
                for line in lines:
                    if 'All reached goal:' in line:
                        print(f"  ✅ SUCCESS: {line.strip()}")
                        return True
                print("  ✅ SUCCESS: Completed")
                return True
            else:
                print(f"  ⚠️  Completed but no clear success indicator")
                return True
        else:
            print(f"  ❌ FAILED (exit {result.returncode})")
            print(f"  stderr: {result.stderr[-200:]}")
            return False
    except subprocess.TimeoutExpired:
        print(f"  ⏱️  TIMEOUT after {timeout}s")
        return False
    except Exception as e:
        print(f"  ❌ EXCEPTION: {e}")
        return False

def main():
    print("="*70)
    print("  FINAL VALIDATION - All Scenarios")
    print("="*70)
    
    results = []
    
    for idx, (scenario, mode, num_robots) in enumerate(TEST_MATRIX, 1):
        success = run_test(scenario, mode, num_robots, idx)
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
        print("\n✅ All scenarios validated!")
        print("   Ready to run full batch of 80 experiments.")
    else:
        print("\n⚠️  Some scenarios failed. Review output above.")
    
    print("="*70)
    
    return 0 if all_passed else 1

if __name__ == '__main__':
    sys.exit(main())
