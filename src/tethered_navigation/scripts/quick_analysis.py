#!/usr/bin/env python3
"""
Quick Results Aggregator - 快速聚合已有实验结果
用于快速生成论文所需图表和数据
"""

import pandas as pd
import numpy as np
from pathlib import Path
import json
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime

RESULTS_DIR = Path('/home/jetson/swarm_simulation_results')

def load_single_exp(exp_dir: Path) -> dict:
    """安全加载单次实验结果"""
    result = {'scenario': None, 'mode': None, 'num_robots': 0, 'success': False,
              'entanglements': 0, 'avg_tension': 0, 'max_tension': 0,
              'traversal_time': 0, 'yielding_events': 0, 'crossing_events': 0,
              'all_at_goal': False, 'duration': 0}
    
    # 解析文件夹名：格式 20260418_114059_bottleneck_cooperative_5robots
    parts = exp_dir.name.split('_')
    # 至少需要：日期 时间 场景 模式 机器人数
    if len(parts) >= 5:
        # parts[0]=日期, parts[1]=时间, parts[2]=场景, parts[3]=模式, parts[4]=机器人数
        result['scenario'] = parts[2]
        result['mode'] = parts[3]
        try:
            result['num_robots'] = int(parts[4].replace('robots', ''))
        except:
            # 尝试解析类似"5robots"的字符串
            for part in parts[4:]:
                if 'robots' in part:
                    try:
                        result['num_robots'] = int(part.replace('robots', ''))
                        break
                    except:
                        pass
    
    # 读取元数据（可选）
    metadata_file = exp_dir / 'metadata.json'
    if metadata_file.exists():
        try:
            with open(metadata_file) as f:
                metadata = json.load(f)
            result['duration'] = metadata.get('total_duration', 0)
            # 如果文件夹名解析失败，从元数据获取
            if result['num_robots'] == 0:
                result['num_robots'] = metadata.get('num_robots', 0)
        except:
            pass
    
    # 读取metrics（安全方式）
    metrics_file = exp_dir / 'swarm_metrics.csv'
    if metrics_file.exists() and metrics_file.stat().st_size > 0:
        try:
            # 跳过可能损坏的行
            df = pd.read_csv(metrics_file, on_bad_lines='skip')
            if not df.empty:
                final = df.iloc[-1]
                
                # 使用get方法安全访问
                result['entanglements'] = int(final.get('total_entanglements', 0))
                result['avg_tension'] = float(final.get('avg_tension', 0))
                result['max_tension'] = float(final.get('max_tension', 0))
                result['traversal_time'] = float(final.get('swarm_traversal_time', -1))
                
                all_at_goal_val = final.get('all_robots_at_goal', False)
                if isinstance(all_at_goal_val, str):
                    result['all_at_goal'] = all_at_goal_val.strip().lower() == 'true'
                else:
                    result['all_at_goal'] = bool(all_at_goal_val)
                
                result['success'] = result['all_at_goal'] and (result['traversal_time'] > 0)
        except Exception as e:
            print(f"  ⚠️  Error reading {exp_dir.name}: {e}")
    
    # 统计事件数
    yielding_file = exp_dir / 'yielding_decisions.csv'
    if yielding_file.exists():
        try:
            df_yield = pd.read_csv(yielding_file, on_bad_lines='skip')
            result['yielding_events'] = len(df_yield)
        except:
            pass
    
    crossing_file = exp_dir / 'crossing_events.csv'
    if crossing_file.exists():
        try:
            df_cross = pd.read_csv(crossing_file, on_bad_lines='skip')
            result['crossing_events'] = len(df_cross)
        except:
            pass
    
    return result

def main():
    print("="*70)
    print("  Quick Swarm Results Analysis")
    print("="*70)
    
    # 收集所有实验
    all_folders = [f for f in RESULTS_DIR.iterdir() if f.is_dir()]
    print(f"\nFound {len(all_folders)} experiment folders")
    
    results = []
    for folder in all_folders:
        try:
            res = load_single_exp(folder)
            results.append(res)
        except Exception as e:
            print(f"  ❌ Failed to load {folder.name}: {e}")
    
    df = pd.DataFrame(results)
    
    # 过滤掉无效数据
    df_valid = df[df['scenario'].notna()].copy()
    print(f"\n✅ Successfully loaded {len(df_valid)} valid experiments")
    
    if len(df_valid) == 0:
        print("❌ No valid data. Exiting.")
        return
    
    # 显示基本信息
    print("\n" + "="*70)
    print("  EXPERIMENT SUMMARY")
    print("="*70)
    print(f"\nScenario distribution:")
    print(df_valid['scenario'].value_counts())
    print(f"\nMode distribution:")
    print(df_valid['mode'].value_counts())
    print(f"\nNumber of robots:")
    print(df_valid['num_robots'].value_counts().sort_index())
    
    # 计算基本统计
    print("\n" + "="*70)
    print("  KEY METRICS")
    print("="*70)
    
    for scenario in df_valid['scenario'].unique():
        print(f"\n{scenario.upper()}:")
        for mode in ['baseline', 'cooperative']:
            mode_data = df_valid[(df_valid['scenario'] == scenario) & (df_valid['mode'] == mode)]
            if len(mode_data) > 0:
                success_rate = mode_data['success'].mean() * 100
                avg_ent = mode_data['entanglements'].mean()
                avg_tension = mode_data['avg_tension'].mean()
                avg_time = mode_data['traversal_time'][mode_data['success']].mean() if any(mode_data['success']) else 0
                avg_yield = mode_data['yielding_events'].mean()
                
                print(f"  {mode:12s}: n={len(mode_data):2d}  "
                      f"Success={success_rate:5.1f}%  "
                      f"Entanglements={avg_ent:5.2f}  "
                      f"Tension={avg_tension:5.1f}N  "
                      f"Time={avg_time:6.1f}s  "
                      f"Yields={avg_yield:5.1f}")
    
    # 保存汇总表
    output_dir = RESULTS_DIR / 'quick_analysis'
    output_dir.mkdir(exist_ok=True)
    
    df_valid.to_csv(output_dir / 'all_experiments.csv', index=False)
    
    print("\n" + "="*70)
    print(f"  Results saved to: {output_dir}")
    print("="*70)
    
    # 生成简单对比图
    try:
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # 成功率对比
        success_by_mode = df_valid.groupby(['scenario', 'mode'])['success'].mean().unstack()
        success_by_mode.plot(kind='bar', ax=axes[0,0])
        axes[0,0].set_ylabel('Success Rate')
        axes[0,0].set_title('Success Rate by Scenario & Mode')
        axes[0,0].legend(title='Mode')
        
        # 纠缠数对比
        ent_by_mode = df_valid.groupby(['scenario', 'mode'])['entanglements'].mean().unstack()
        ent_by_mode.plot(kind='bar', ax=axes[0,1])
        axes[0,1].set_ylabel('Average Entanglements')
        axes[0,1].set_title('Entanglement Count')
        axes[0,1].legend(title='Mode')
        
        # 张力对比
        tension_by_mode = df_valid.groupby(['scenario', 'mode'])['avg_tension'].mean().unstack()
        tension_by_mode.plot(kind='bar', ax=axes[1,0])
        axes[1,0].set_ylabel('Average Tension (N)')
        axes[1,0].set_title('Tether Tension')
        axes[1,0].legend(title='Mode')
        
        # 穿越时间对比
        time_by_mode = df_valid[df_valid['success']].groupby(['scenario', 'mode'])['traversal_time'].mean().unstack()
        if not time_by_mode.empty:
            time_by_mode.plot(kind='bar', ax=axes[1,1])
            axes[1,1].set_ylabel('Average Traversal Time (s)')
            axes[1,1].set_title('Traversal Time (Success Only)')
            axes[1,1].legend(title='Mode')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'summary_plots.png', dpi=150)
        plt.close()
        print(f"✅ Plots saved to: {output_dir}/summary_plots.png")
    except Exception as e:
        print(f"⚠️  Plotting error: {e}")

if __name__ == '__main__':
    main()
