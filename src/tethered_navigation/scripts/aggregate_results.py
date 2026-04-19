#!/usr/bin/env python3
"""
Aggregate Results for Swarm Robotics Paper
===========================================
从所有实验文件夹中提取关键指标，生成论文表格数据

生成：
1. 成功率对比表 (Baseline vs Cooperative)
2. 纠缠率统计
3. 穿越时间对比
4. 张力分布
5. 涌现行为证据
"""

import pandas as pd
import numpy as np
import json
from pathlib import Path
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
import seaborn as sns

RESULTS_DIR = Path('/home/jetson/swarm_simulation_results')

def load_experiment_results(exp_folder: Path) -> Dict:
    """加载单次实验的结果"""
    result = {
        'folder': exp_folder.name,
        'scenario': None,
        'mode': None,
        'num_robots': 0,
        'success': False,
        'entanglements': 0,
        'avg_tension': 0.0,
        'max_tension': 0.0,
        'traversal_time': 0.0,
        'yielding_events': 0,
        'crossing_events': 0,
        'all_at_goal': False,
        'duration': 0.0
    }
    
    # 从文件夹名解析
    # 格式: YYYYMMDD_HHMMSS_scenario_mode_numrobots
    parts = exp_folder.name.split('_')
    if len(parts) >= 5:
        result['scenario'] = parts[2]  # scenario是第3部分
        result['mode'] = parts[3]      # mode是第4部分
        try:
            result['num_robots'] = int(parts[4].replace('robots', ''))
        except:
            pass
    
    # 读取元数据
    metadata_file = exp_folder / 'metadata.json'
    if metadata_file.exists():
        with open(metadata_file) as f:
            metadata = json.load(f)
        result['duration'] = metadata.get('total_duration', 0)
        result['num_robots'] = metadata.get('num_robots', result['num_robots'])
    
    # 读取swarm_metrics
    metrics_file = exp_folder / 'swarm_metrics.csv'
    if metrics_file.exists():
        import pandas as pd
        try:
            # 读取CSV，跳过格式错误的行
            df = pd.read_csv(metrics_file, on_bad_lines='skip')

            if not df.empty:
                # 清理数据：移除全为NaN的行，以及行数少于预期列数的行
                expected_cols = ['timestamp', 'active_robots', 'total_entanglements',
                                'resolved_entanglements', 'avg_tension', 'max_tension',
                                'swarm_traversal_time', 'all_at_goal']

                # 过滤掉数值类型不正确的行
                valid_rows = []
                for idx, row in df.iterrows():
                    try:
                        # 确保关键数值列能转换为float
                        float(row['timestamp'])
                        int(row.get('total_entanglements', 0))
                        valid_rows.append(row)
                    except (ValueError, TypeError):
                        continue

                if valid_rows:
                    df_clean = pd.DataFrame(valid_rows).reset_index(drop=True)
                    final = df_clean.iloc[-1]

                    # 安全访问列
                    def get_col_value(row, key, default=0, cast_type=float):
                        for col in row.index:
                            if col.lower().strip() == key.lower().strip():
                                val = row[col]
                                if pd.isna(val):
                                    return default
                                try:
                                    return cast_type(val)
                                except:
                                    return default
                        return default

                    result['entanglements'] = int(get_col_value(final, 'total_entanglements', 0, int))
                    result['avg_tension'] = get_col_value(final, 'avg_tension', 0.0, float)
                    result['max_tension'] = get_col_value(final, 'max_tension', 0.0, float)
                    result['traversal_time'] = get_col_value(final, 'swarm_traversal_time', 0.0, float)

                    # 处理all_robots_at_goal
                    all_at_goal_val = get_col_value(final, 'all_at_goal', False)
                    if isinstance(all_at_goal_val, str):
                        result['all_at_goal'] = all_at_goal_val.strip().lower() == 'true'
                    else:
                        result['all_at_goal'] = bool(all_at_goal_val)

                    result['success'] = result['all_at_goal'] and (result['traversal_time'] > 0)
                else:
                    print(f"  ⚠️  No valid data rows in {exp_folder.name}")
            else:
                print(f"  ⚠️  Empty metrics file: {exp_folder.name}")
        except Exception as e:
            print(f"  ⚠️  Error reading metrics: {e}")
    
    # 读取yielding_decisions和crossing_events计数
    yielding_file = exp_folder / 'yielding_decisions.csv'
    if yielding_file.exists():
        df_yield = pd.read_csv(yielding_file)
        result['yielding_events'] = len(df_yield)
    
    crossing_file = exp_folder / 'crossing_events.csv'
    if crossing_file.exists():
        df_cross = pd.read_csv(crossing_file)
        result['crossing_events'] = len(df_cross)
    
    return result

def collect_all_experiments() -> pd.DataFrame:
    """收集所有实验结果到DataFrame"""
    all_folders = [f for f in RESULTS_DIR.iterdir() if f.is_dir()]
    
    results = []
    for folder in all_folders:
        try:
            res = load_experiment_results(folder)
            results.append(res)
        except Exception as e:
            print(f"  ⚠️  Error loading {folder.name}: {e}")
    
    df = pd.DataFrame(results)
    print(f"Loaded {len(df)} experiments")
    return df

def compute_summary_statistics(df: pd.DataFrame) -> Dict:
    """计算论文所需的汇总统计"""
    summary = {}
    
    # 按场景和模式分组
    for scenario in df['scenario'].unique():
        if not scenario:
            continue
        
        scenario_data = df[df['scenario'] == scenario]
        
        for mode in ['baseline', 'cooperative']:
            mode_data = scenario_data[scenario_data['mode'] == mode]
            
            if len(mode_data) == 0:
                continue
            
            key = f"{scenario}_{mode}"
            
            # 成功率
            success_rate = mode_data['success'].mean() * 100
            
            # 平均纠缠数
            avg_entanglements = mode_data['entanglements'].mean()
            
            # 纠缠率（每小时）- 假设每实验持续300秒
            duration_hours = mode_data['duration'].mean() / 3600.0
            entanglements_per_hour = avg_entanglements / duration_hours if duration_hours > 0 else 0
            
            # 平均穿越时间
            successful_trials = mode_data[mode_data['success']]
            avg_traversal_time = successful_trials['traversal_time'].mean() if len(successful_trials) > 0 else 0
            
            # 平均张力
            avg_tension = mode_data['avg_tension'].mean()
            max_tension = mode_data['max_tension'].mean()
            
            # Yielding事件统计
            avg_yielding = mode_data['yielding_events'].mean()
            
            summary[key] = {
                'num_trials': len(mode_data),
                'success_rate_pct': success_rate,
                'avg_entanglements': avg_entanglements,
                'entanglements_per_hour': entanglements_per_hour,
                'avg_traversal_time_s': avg_traversal_time,
                'avg_tension_N': avg_tension,
                'max_tension_N': max_tension,
                'avg_yielding_events': avg_yielding
            }
    
    return summary

def generate_paper_table(summary: Dict, output_dir: Path):
    """生成论文表格（LaTeX格式）"""
    
    # 表1：主要结果对比
    table1 = """
\\begin{table}[ht]
\\centering
\\caption{Swarm Coordination Results: Baseline vs Cooperative}
\\label{table:swarm_results}
\\begin{tabular}{lccccc}
\\toprule
Scenario & Mode & Success Rate & Entanglements & Traversal Time & Avg Tension (N) \\\\
\\midrule
"""
    
    scenarios = ['bottleneck', 'crossing', 'expansion']
    for scenario in scenarios:
        for mode in ['baseline', 'cooperative']:
            key = f"{scenario}_{mode}"
            if key in summary:
                s = summary[key]
                table1 += f"{scenario.capitalize()} & {mode.capitalize()} & "
                table1 += f"{s['success_rate_pct']:.1f}\\% & "
                table1 += f"{s['avg_entanglements']:.2f} & "
                table1 += f"{s['avg_traversal_time_s']:.1f}s & "
                table1 += f"{s['avg_tension_N']:.1f} \\\\\n"
    
    table1 += """\\bottomrule
\\end{tabular}
\\end{table}
"""
    
    with open(output_dir / 'paper_table1.tex', 'w') as f:
        f.write(table1)
    
    print("✅ Generated LaTeX table: paper_table1.tex")
    
    # 生成CSV摘要（供Excel使用）
    rows = []
    for scenario in scenarios:
        for mode in ['baseline', 'cooperative']:
            key = f"{scenario}_{mode}"
            if key in summary:
                s = summary[key]
                rows.append({
                    'Scenario': scenario,
                    'Mode': mode,
                    'Success Rate (%)': round(s['success_rate_pct'], 2),
                    'Avg Entanglements': round(s['avg_entanglements'], 2),
                    'Entanglements/Hour': round(s['entanglements_per_hour'], 2),
                    'Avg Traversal Time (s)': round(s['avg_traversal_time_s'], 2),
                    'Avg Tension (N)': round(s['avg_tension_N'], 2),
                    'Max Tension (N)': round(s['max_tension_N'], 2),
                    'Yielding Events': round(s['avg_yielding_events'], 2)
                })
    
    df_summary = pd.DataFrame(rows)
    df_summary.to_csv(output_dir / 'summary_statistics.csv', index=False)
    print("✅ Generated CSV summary: summary_statistics.csv")
    
    return df_summary

def generate_plots(df: pd.DataFrame, output_dir: Path):
    """生成论文所需的图表"""
    
    # 图1：成功率对比（条形图）
    fig, ax = plt.subplots(figsize=(10, 6))
    success_by_scenario = df.groupby(['scenario', 'mode'])['success'].mean() * 100
    success_by_scenario.unstack().plot(kind='bar', ax=ax)
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('Swarm Coordination Success Rate: Baseline vs Cooperative')
    ax.legend(title='Mode')
    plt.xticks(rotation=0)
    plt.tight_layout()
    plt.savefig(output_dir / 'figure_success_rate.png', dpi=300)
    plt.close()
    
    # 图2：纠缠数对比
    fig, ax = plt.subplots(figsize=(10, 6))
    entanglements = df.groupby(['scenario', 'mode'])['entanglements'].mean()
    entanglements.unstack().plot(kind='bar', ax=ax)
    ax.set_ylabel('Average Entanglements per Trial')
    ax.set_title('Inter-Robot Entanglement Count')
    ax.legend(title='Mode')
    plt.xticks(rotation=0)
    plt.tight_layout()
    plt.savefig(output_dir / 'figure_entanglements.png', dpi=300)
    plt.close()
    
    # 图3：张力分布（箱线图）
    fig, ax = plt.subplots(figsize=(10, 6))
    data_box = [df[df['mode'] == 'baseline']['avg_tension'],
                df[df['mode'] == 'cooperative']['avg_tension']]
    ax.boxplot(data_box, labels=['Baseline', 'Cooperative'])
    ax.set_ylabel('Average Tension (N)')
    ax.set_title('Tether Tension Distribution')
    plt.tight_layout()
    plt.savefig(output_dir / 'figure_tension_boxplot.png', dpi=300)
    plt.close()
    
    # 图4：穿越时间对比
    fig, ax = plt.subplots(figsize=(10, 6))
    time_data = df.groupby(['scenario', 'mode'])['traversal_time'].mean()
    time_data.unstack().plot(kind='bar', ax=ax)
    ax.set_ylabel('Average Traversal Time (s)')
    ax.set_title('Swarm Traversal Time Comparison')
    ax.legend(title='Mode')
    plt.xticks(rotation=0)
    plt.tight_layout()
    plt.savefig(output_dir / 'figure_traversal_time.png', dpi=300)
    plt.close()
    
    print("✅ Generated 4 plots for paper:")
    print(f"   - {output_dir}/figure_success_rate.png")
    print(f"   - {output_dir}/figure_entanglements.png")
    print(f"   - {output_dir}/figure_tension_boxplot.png")
    print(f"   - {output_dir}/figure_traversal_time.png")

def generate_emergent_behavior_example(results_dir: Path):
    """从一个成功的cooperative实验中提取涌现行为轨迹图"""
    
    # 找一个好的cooperative实验（成功且有一定纠缠）
    cooperative_success = []
    for exp_dir in RESULTS_DIR.iterdir():
        if not exp_dir.is_dir():
            continue
        if 'cooperative' not in exp_dir.name:
            continue
        
        try:
            res = load_experiment_results(exp_dir)
            if res['success'] and res['yielding_events'] > 0:
                cooperative_success.append(exp_dir)
        except:
            pass
    
    if not cooperative_success:
        print("⚠️  No successful cooperative experiments with yielding found")
        return
    
    # 选第一个
    example_dir = cooperative_success[0]
    print(f"\n📊 Generating emergent behavior plot from: {example_dir.name}")
    
    # 加载轨迹
    trajectories = {}
    for traj_file in example_dir.glob('trajectory_robot_*.csv'):
        robot_id = traj_file.stem.split('_')[1]
        trajectories[robot_id] = pd.read_csv(traj_file)
    
    # 绘制轨迹
    fig, ax = plt.subplots(figsize=(12, 10))
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(trajectories)))
    
    for idx, (robot_id, traj) in enumerate(trajectories.items()):
        # 绘制路径
        ax.plot(traj['x'], traj['y'], '-', color=colors[idx], linewidth=1.5, alpha=0.7, label=f'{robot_id}')
        # 起点
        ax.plot(traj['x'].iloc[0], traj['y'].iloc[0], 'o', color=colors[idx], markersize=10, markeredgecolor='black')
        # 终点
        ax.plot(traj['x'].iloc[-1], traj['y'].iloc[-1], 's', color=colors[idx], markersize=10, markeredgecolor='black')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Emergent Yielding Behavior: {example_dir.name}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    output_path = results_dir / 'emergent_behavior_trajectories.png'
    plt.savefig(output_path, dpi=300)
    plt.close()
    
    print(f"✅ Saved emergent behavior plot: {output_path}")

def main():
    print("="*70)
    print("  Swarm Results Aggregation")
    print("="*70)
    
    # 1. 收集所有数据
    print("\n📊 Collecting experiment data...")
    df = collect_all_experiments()
    
    if len(df) == 0:
        print("❌ No experiment data found!")
        return
    
    print(f"   Found {len(df)} experiments")
    print(f"   Scenarios: {df['scenario'].unique()}")
    print(f"   Modes: {df['mode'].unique()}")
    
    # 2. 计算统计
    print("\n📈 Computing summary statistics...")
    summary = compute_summary_statistics(df)
    
    # 打印到控制台
    print("\n" + "="*70)
    print("  SUMMARY STATISTICS")
    print("="*70)
    for key, s in summary.items():
        print(f"\n{key}:")
        print(f"  Trials:          {s['num_trials']}")
        print(f"  Success Rate:    {s['success_rate_pct']:.1f}%")
        print(f"  Entanglements:   {s['avg_entanglements']:.2f} (per hour: {s['entanglements_per_hour']:.1f})")
        print(f"  Traversal Time:  {s['avg_traversal_time_s']:.1f}s")
        print(f"  Avg Tension:     {s['avg_tension_N']:.1f}N")
    
    # 3. 生成论文表格
    output_dir = RESULTS_DIR / 'aggregated_results'
    output_dir.mkdir(exist_ok=True)
    
    print("\n📝 Generating paper tables...")
    df_summary = generate_paper_table(summary, output_dir)
    
    # 4. 生成图表
    print("\n📊 Generating plots...")
    generate_plots(df, output_dir)
    
    # 5. 涌现行为示例
    print("\n🎯 Generating emergent behavior example...")
    generate_emergent_behavior_example(output_dir)
    
    # 6. 计算改进幅度
    print("\n" + "="*70)
    print("  IMPROVEMENT METRICS (Cooperative vs Baseline)")
    print("="*70)
    
    for scenario in ['bottleneck', 'crossing', 'expansion']:
        baseline_key = f"{scenario}_baseline"
        coop_key = f"{scenario}_cooperative"
        
        if baseline_key in summary and coop_key in summary:
            b = summary[baseline_key]
            c = summary[coop_key]
            
            success_improvement = c['success_rate_pct'] - b['success_rate_pct']
            entanglement_reduction = b['avg_entanglements'] - c['avg_entanglements']
            entanglement_pct_reduction = (entanglement_reduction / b['avg_entanglements'] * 100) if b['avg_entanglements'] > 0 else 0
            
            print(f"\n{scenario.upper()}:")
            print(f"  Success rate improvement:  {success_improvement:+.1f}%")
            print(f"  Entanglement reduction:    {entanglement_reduction:.2f} ({entanglement_pct_reduction:.1f}%)")
            print(f"  Tension reduction:         {b['avg_tension_N'] - c['avg_tension_N']:.1f}N")
    
    print("\n" + "="*70)
    print(f"  All results saved to: {output_dir}")
    print("="*70)
    
    # 保存完整数据
    df.to_csv(output_dir / 'all_experiments.csv', index=False)
    print(f"\n✅ Complete dataset: {output_dir / 'all_experiments.csv'}")

if __name__ == '__main__':
    main()
