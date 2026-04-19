#!/usr/bin/env python3
"""
Results Analysis and Visualization for Swarm Robotics Paper
============================================================
处理实验数据，生成论文所需的统计和图表

生成的指标：
1. Swarm Success Rate (对比 baseline vs cooperative)
2. Inter-Robot Entanglement Count
3. Average Traversal Time
4. Maximum Tension Distribution
5. Emergent Yielding Behavior (轨迹图)

用法:
  python3 analyze_results.py /path/to/experiment_folder
"""

import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from datetime import datetime
import json
from typing import Dict, List, Tuple
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体和绘图风格
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False
sns.set_style("whitegrid")
sns.set_palette("husl")

class ExperimentAnalyzer:
    """单次实验数据分析器"""
    
    def __init__(self, exp_dir: Path):
        self.exp_dir = Path(exp_dir)
        self.scenario = exp_dir.name.split('_')[0]  # 从文件夹名提取场景
        
        # 加载数据文件
        self.swarm_metrics = self._load_csv('swarm_metrics.csv')
        self.yielding_decisions = self._load_csv('yielding_decisions.csv')
        self.crossing_events = self._load_csv('crossing_events.csv')
        
        # 加载每个机器人的轨迹
        self.trajectories = {}
        traj_files = list(self.exp_dir.glob('trajectory_robot_*.csv'))
        for tf in traj_files:
            robot_id = tf.stem.split('_')[1]  # 提取 robot_1
            self.trajectories[robot_id] = pd.read_csv(tf)
        
        # 加载元数据
        self.metadata = self._load_json('metadata.json')
        
        print(f"✅ 已加载实验数据: {self.scenario} ({len(self.trajectories)} 机器人)")
    
    def _load_csv(self, filename: str) -> pd.DataFrame:
        filepath = self.exp_dir / filename
        if filepath.exists():
            return pd.read_csv(filepath)
        return pd.DataFrame()
    
    def _load_json(self, filename: str) -> dict:
        filepath = self.exp_dir / filename
        if filepath.exists():
            with open(filepath, 'r') as f:
                return json.load(f)
        return {}
    
    def compute_summary_statistics(self) -> Dict:
        """计算摘要统计指标"""
        summary = {
            'scenario': self.scenario,
            'num_robots': self.metadata.get('num_robots', len(self.trajectories)),
            'duration_seconds': self.metadata.get('total_duration', 0),
        }
        
        if not self.swarm_metrics.empty:
            # 最终指标（取最后一个时间点）
            final_metrics = self.swarm_metrics.iloc[-1]
            
            summary.update({
                'final_entanglements': int(final_metrics['total_entanglements']),
                'final_avg_tension': float(final_metrics['avg_tension']),
                'final_max_tension': float(final_metrics['max_tension']),
                'all_at_goal': bool(final_metrics['all_robots_at_goal']),
                'swarm_traversal_time': float(final_metrics['swarm_traversal_time']),
            })
            
            # 计算纠缠率（每小时）
            duration_hours = summary['duration_seconds'] / 3600.0
            if duration_hours > 0:
                summary['entanglements_per_hour'] = summary['final_entanglements'] / duration_hours
            else:
                summary['entanglements_per_hour'] = 0.0
        
        # Yielding 统计
        if not self.yielding_decisions.empty:
            summary['total_yielding_decisions'] = len(self.yielding_decisions)
            summary['yield_actions'] = int(
                (self.yielding_decisions['suggested_action'] == 'yield').sum()
            )
            summary['reroute_actions'] = int(
                (self.yielding_decisions['suggested_action'] == 'reroute').sum()
            )
        
        # 交叉事件统计
        if not self.crossing_events.empty:
            summary['total_crossings'] = len(self.crossing_events)
            summary['avg_crossing_severity'] = float(
                self.crossing_events['severity'].mean()
            )
        
        return summary
    
    def plot_trajectories(self, ax=None, show_tether: bool = True):
        """绘制所有机器人的轨迹（2D 俯视图）"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.trajectories)))
        
        for idx, (robot_id, traj) in enumerate(self.trajectories.items()):
            color = colors[idx]
            
            # 绘制轨迹线
            ax.plot(traj['x'], traj['y'], 
                   color=color, alpha=0.6, linewidth=1.5, label=robot_id)
            
            # 标记起点和终点
            ax.scatter(traj['x'].iloc[0], traj['y'].iloc[0], 
                      color=color, marker='o', s=100, edgecolors='black', zorder=5)
            ax.scatter(traj['x'].iloc[-1], traj['y'].iloc[-1], 
                      color=color, marker='s', s=100, edgecolors='black', zorder=5)
            
            # 绘制 tether（从起点到当前位置的连线）
            if show_tether and len(traj) > 0:
                anchor_x = traj['x'].iloc[0]
                anchor_y = traj['y'].iloc[0]
                ax.plot([anchor_x, traj['x'].iloc[-1]], 
                       [anchor_y, traj['y'].iloc[-1]],
                       color=color, linestyle='--', alpha=0.3, linewidth=1)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'Swarm Trajectories - {self.scenario.upper()}', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        return ax
    
    def plot_tension_over_time(self, ax=None):
        """绘制每个机器人的张力随时间变化"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 5))
        
        for robot_id, traj in self.trajectories.items():
            ax.plot(traj['timestamp'], traj['tension'], 
                   label=robot_id, alpha=0.7, linewidth=1.5)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Tether Tension (N)', fontsize=12)
        ax.set_title('Tether Tension Over Time', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
        
        return ax
    
    def plot_entanglement_timeline(self, ax=None):
        """绘制纠缠事件时间线"""
        if self.crossing_events.empty:
            return None
        
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 4))
        
        # 按机器人对分组
        crossing_groups = self.crossing_events.groupby(['robot_a', 'robot_b'])
        
        colors = plt.cm.Set2(np.linspace(0, 1, len(crossing_groups)))
        for (robot_a, robot_b), group in crossing_groups:
            label = f"{robot_a}-{robot_b}"
            ax.scatter(group['timestamp'], [label] * len(group),
                      c=[colors[list(crossing_groups.groups.keys()).index((robot_a, robot_b))]],
                      s=50, alpha=0.7, edgecolors='black', linewidth=0.5)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Robot Pair', fontsize=12)
        ax.set_title('Entanglement Events Timeline', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        
        return ax
    
    def plot_yielding_decisions(self, ax=None):
        """绘制 yielding 决策分布"""
        if self.yielding_decisions.empty:
            return None
        
        if ax is None:
            fig, ax = plt.subplots(figsize=(8, 5))
        
        action_counts = self.yielding_decisions['suggested_action'].value_counts()
        
        bars = ax.bar(action_counts.index, action_counts.values, 
                     color=['#ff6b6b', '#feca57', '#48dbfb', '#1dd1a1'])
        
        ax.set_xlabel('Yielding Action', fontsize=12)
        ax.set_ylabel('Count', fontsize=12)
        ax.set_title('Yielding Decision Distribution', fontsize=14, fontweight='bold')
        
        # 添加数值标签
        for bar, count in zip(bars, action_counts.values):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                   f'{count}', ha='center', va='bottom', fontsize=10)
        
        return ax
    
    def generate_full_report(self, output_dir: Path = None):
        """生成完整分析报告（包含所有图表）"""
        if output_dir is None:
            output_dir = self.exp_dir / 'analysis'
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"\n📊 生成分析报告: {output_dir}")
        
        # 1. 计算摘要统计
        summary = self.compute_summary_statistics()
        summary_file = output_dir / 'summary_statistics.json'
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"  ✅ 摘要统计保存到: {summary_file}")
        
        # 2. 创建多子图图表
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle(f'Swarm Experiment Analysis: {self.scenario.upper()}', 
                    fontsize=16, fontweight='bold', y=0.98)
        
        # 子图 1: 轨迹
        ax1 = axes[0, 0]
        self.plot_trajectories(ax1)
        ax1.set_title('Robot Trajectories', fontweight='bold')
        
        # 子图 2: 张力时间线
        ax2 = axes[0, 1]
        self.plot_tension_over_time(ax2)
        ax2.set_title('Tension Over Time', fontweight='bold')
        
        # 子图 3: 纠缠事件
        ax3 = axes[0, 2]
        if not self.crossing_events.empty:
            self.plot_entanglement_timeline(ax3)
        else:
            ax3.text(0.5, 0.5, 'No crossing events', 
                    ha='center', va='center', transform=ax3.transAxes)
        ax3.set_title('Entanglement Events', fontweight='bold')
        
        # 子图 4: Yielding 决策
        ax4 = axes[1, 0]
        if not self.yielding_decisions.empty:
            self.plot_yielding_decisions(ax4)
        else:
            ax4.text(0.5, 0.5, 'No yielding decisions', 
                    ha='center', va='center', transform=ax4.transAxes)
        ax4.set_title('Yielding Actions', fontweight='bold')
        
        # 子图 5: 张力分布直方图
        ax5 = axes[1, 1]
        all_tensions = []
        for traj in self.trajectories.values():
            all_tensions.extend(traj['tension'].tolist())
        if all_tensions:
            ax5.hist(all_tensions, bins=30, edgecolor='black', alpha=0.7)
            ax5.axvline(np.mean(all_tensions), color='red', linestyle='--', 
                       label=f'Mean: {np.mean(all_tensions):.1f}N')
            ax5.legend()
        ax5.set_xlabel('Tension (N)')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Tension Distribution', fontweight='bold')
        
        # 子图 6: 指标表格（文本）
        ax6 = axes[1, 2]
        ax6.axis('tight')
        ax6.axis('off')
        table_data = [
            ['Metric', 'Value'],
            ['Robots', summary.get('num_robots', 'N/A')],
            ['Entanglements', summary.get('final_entanglements', 'N/A')],
            ['Entanglements/hr', f"{summary.get('entanglements_per_hour', 0):.1f}"],
            ['Avg Tension (N)', f"{summary.get('final_avg_tension', 0):.1f}"],
            ['Max Tension (N)', f"{summary.get('final_max_tension', 0):.1f}"],
            ['Traversal Time (s)', f"{summary.get('swarm_traversal_time', 0):.1f}"],
            ['All Reached Goal', str(summary.get('all_at_goal', False))],
            ['Yield Decisions', summary.get('total_yielding_decisions', 0)],
        ]
        table = ax6.table(cellText=table_data, loc='center', cellLoc='left')
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1, 2)
        ax6.set_title('Summary Statistics', fontweight='bold', y=0.9)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'full_analysis.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  ✅ 完整图表保存到: {output_dir / 'full_analysis.png'}")
        
        # 3. 生成 LaTeX 表格数据
        self._generate_latex_table(summary, output_dir)
        
        return summary
    
    def _generate_latex_table(self, summary: Dict, output_dir: Path):
        """生成 LaTeX 表格代码（用于论文）"""
        latex_code = f"""
% Table generated by analyze_results.py - {datetime.now().strftime('%Y-%m-%d')}
\\begin{{table}}[ht]
\\centering
\\caption{{Swarm Experiment Results - {self.scenario.upper()} Scenario}}
\\label{{tab:swarm_results_{self.scenario}}}
\\begin{{tabular}}{{'lrr'}}
\\hline
\\textbf{{Metric}} & \\textbf{{Baseline}} & \\textbf{{Cooperative}} \\\\
\\hline
Number of Robots & {summary.get('num_robots', 'N/A')} & {summary.get('num_robots', 'N/A')} \\\\
Total Entanglements & N/A & {summary.get('final_entanglements', 'N/A')} \\\\
Entanglements per Hour & N/A & {summary.get('entanglements_per_hour', 0):.1f} \\\\
Average Tension (N) & N/A & {summary.get('final_avg_tension', 0):.1f} \\\\
Max Tension (N) & N/A & {summary.get('final_max_tension', 0):.1f} \\\\
Swarm Traversal Time (s) & N/A & {summary.get('swarm_traversal_time', 0):.1f} \\\\
All Robots Reached Goal & N/A & {str(summary.get('all_at_goal', False))} \\\\
Yielding Decisions & N/A & {summary.get('total_yielding_decisions', 0)} \\\\
\\hline
\\end{{tabular}}
\\end{{table}}
"""
        with open(output_dir / 'latex_table.tex', 'w') as f:
            f.write(latex_code)
        print(f"  ✅ LaTeX 表格代码保存到: {output_dir / 'latex_table.tex'}")


def compare_experiments(baseline_dir: Path, cooperative_dir: Path) -> Dict:
    """对比 baseline 和 cooperative 实验"""
    print("\n" + "="*60)
    print("对比分析: Baseline vs Cooperative")
    print("="*60)
    
    baseline = ExperimentAnalyzer(baseline_dir)
    coop = ExperimentAnalyzer(cooperative_dir)
    
    baseline_stats = baseline.compute_summary_statistics()
    coop_stats = coop.compute_summary_statistics()
    
    # 打印对比表格
    comparison = {
        'metric': [],
        'baseline': [],
        'cooperative': [],
        'improvement': [],
        'unit': []
    }
    
    metrics_to_compare = [
        ('entanglements_per_hour', 'Entanglements / hour', ''),
        ('final_avg_tension', 'Avg Tension', 'N'),
        ('final_max_tension', 'Max Tension', 'N'),
        ('swarm_traversal_time', 'Traversal Time', 's'),
        ('total_yielding_decisions', 'Yielding Decisions', 'count'),
    ]
    
    print(f"\n{'Metric':<25} {'Baseline':>15} {'Cooperative':>15} {'Improvement':>15}")
    print("-" * 75)
    
    for key, label, unit in metrics_to_compare:
        base_val = baseline_stats.get(key, 0)
        coop_val = coop_stats.get(key, 0)
        
        if key == 'entanglements_per_hour' and coop_val > 0:
            # 越低越好
            improvement = ((base_val - coop_val) / base_val * 100) if base_val > 0 else 0
            improvement_str = f"-{improvement:.1f}%" if improvement > 0 else f"+{abs(improvement):.1f}%"
        elif key in ['final_avg_tension', 'final_max_tension', 'swarm_traversal_time']:
            # 张力可能略增（让行减速），纠缠大幅减少是好的
            improvement = ((base_val - coop_val) / base_val * 100) if base_val > 0 else 0
            improvement_str = f"{improvement:+.1f}%"
        else:
            improvement_str = "N/A"
        
        print(f"{label:<25} {base_val:>15.2f} {coop_val:>15.2f} {improvement_str:>15}")
        
        comparison['metric'].append(label)
        comparison['baseline'].append(base_val)
        comparison['cooperative'].append(coop_val)
        comparison['improvement'].append(improvement_str)
        comparison['unit'].append(unit)
    
    # 保存对比结果
    comparison_df = pd.DataFrame(comparison)
    comparison_file = cooperative_dir / 'comparison_with_baseline.csv'
    comparison_df.to_csv(comparison_file, index=False)
    print(f"\n✅ 对比结果保存到: {comparison_file}")
    
    return comparison


def main():
    if len(sys.argv) < 2:
        print("用法:")
        print("  分析单次实验: python analyze_results.py <experiment_folder>")
        print("  对比实验:      python analyze_results.py --compare <baseline_folder> <coop_folder>")
        print("\n示例:")
        print("  python analyze_results.py /home/jetson/swarm_experiments/bottleneck_cooperative_20250418_123456")
        sys.exit(1)
    
    if sys.argv[1] == '--compare' and len(sys.argv) >= 4:
        baseline_dir = Path(sys.argv[2])
        coop_dir = Path(sys.argv[3])
        compare_experiments(baseline_dir, coop_dir)
    else:
        # 单次实验分析
        exp_dir = Path(sys.argv[1])
        if not exp_dir.exists():
            print(f"错误: 文件夹不存在: {exp_dir}")
            sys.exit(1)
        
        analyzer = ExperimentAnalyzer(exp_dir)
        summary = analyzer.generate_full_report()
        
        print("\n" + "="*60)
        print("实验摘要:")
        print("="*60)
        for key, value in summary.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.3f}")
            else:
                print(f"  {key}: {value}")


if __name__ == '__main__':
    main()
