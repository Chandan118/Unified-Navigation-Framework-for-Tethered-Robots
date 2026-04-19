#!/usr/bin/env python3
"""
Multi-Robot Simulation Manager for Swarm Experiments
=====================================================
管理多机器人仿真的启动、协调和数据记录

支持两种模式：
1. Baseline: 机器人独立运行，无 swarm 通信
2. Cooperative: 机器人使用 swarm_tether_states 进行协调

为简化部署，使用2D仿真（基于 nav2 和自定义 obstacles）
可在 Jetson Orin Nano 上运行
"""

import os
import sys
import time
import json
import csv
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Tuple, Optional
import argparse
import subprocess
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from tethered_navigation.msg import SwarmMetrics, TetherState
from sensor_msgs.msg import LaserScan


class SimulationDataLogger:
    """收集和保存实验数据"""
    
    def __init__(self, log_dir: str, scenario_name: str, mode: str):
        self.log_dir = Path(log_dir) / scenario_name / mode
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # 创建数据文件
        self.swarm_metrics_file = open(self.log_dir / 'swarm_metrics.csv', 'w', newline='')
        self.swarm_writer = csv.writer(self.swarm_metrics_file)
        self.swarm_writer.writerow([
            'timestamp', 'active_robots', 'total_entanglements',
            'avg_tension', 'max_tension', 'swarm_traversal_time', 'all_at_goal'
        ])
        
        self.robot_trajectories_file = open(self.log_dir / 'robot_trajectories.csv', 'w', newline='')
        self.traj_writer = csv.writer(self.robot_trajectories_file)
        self.traj_writer.writerow([
            'robot_id', 'timestamp', 'x', 'y', 'theta',
            'tension', 'velocity', 'yielding_action'
        ])
        
        self.crossing_events_file = open(self.log_dir / 'crossing_events.csv', 'w', newline='')
        self.crossing_writer = csv.writer(self.crossing_events_file)
        self.crossing_writer.writerow([
            'timestamp', 'robot_a', 'robot_b', 'crossing_x', 'crossing_y', 'severity'
        ])
        
        # 实验元数据
        self.metadata = {
            'scenario': scenario_name,
            'mode': mode,
            'num_robots': 0,
            'start_time': None,
            'end_time': None,
            'seed': None
        }
        
        self.start_time = None
    
    def start_experiment(self, num_robots: int, seed: int = 42):
        self.start_time = time.time()
        self.metadata['num_robots'] = num_robots
        self.metadata['seed'] = seed
        self.metadata['start_time'] = datetime.now().isoformat()
    
    def log_swarm_metrics(self, timestamp: float, metrics: SwarmMetrics):
        self.swarm_writer.writerow([
            timestamp,
            metrics.active_robots,
            metrics.total_entanglements,
            metrics.avg_tension,
            metrics.max_tension,
            metrics.swarm_traversal_time,
            metrics.all_robots_at_goal
        ])
    
    def log_robot_pose(self, robot_id: str, x: float, y: float, theta: float,
                       tension: float, velocity: float, yielding_action: str):
        self.traj_writer.writerow([
            time.time() - self.start_time if self.start_time else 0.0,
            robot_id, x, y, theta, tension, velocity, yielding_action
        ])
    
    def log_crossing_event(self, robot_a: str, robot_b: str, 
                          crossing_x: float, crossing_y: float, severity: float):
        self.crossing_writer.writerow([
            time.time() - self.start_time if self.start_time else 0.0,
            robot_a, robot_b, crossing_x, crossing_y, severity
        ])
    
    def end_experiment(self):
        end_time = time.time()
        self.metadata['end_time'] = datetime.now().isoformat()
        self.metadata['total_duration'] = end_time - self.start_time if self.start_time else 0
        
        # 保存元数据
        with open(self.log_dir / 'metadata.json', 'w') as f:
            json.dump(self.metadata, f, indent=2)
        
        # 关闭文件
        self.swarm_metrics_file.close()
        self.robot_trajectories_file.close()
        self.crossing_events_file.close()
        
        print(f"\n✅ 实验数据已保存到: {self.log_dir}")


class SimulationScenario:
    """仿真场景基类"""
    
    def __init__(self, name: str, num_robots: int = 5):
        self.name = name
        self.num_robots = num_robots
        self.robot_start_poses = []  # List of (x, y, yaw)
        self.robot_goals = []        # List of (x, y)
        
    def generate_robot_configs(self) -> Tuple[List[Tuple], List[Tuple]]:
        """
        生成机器人的起始位置和目标位置
        Returns:
            start_poses: [(x, y, yaw), ...]
            goals: [(x, y), ...]
        """
        raise NotImplementedError


class BottleneckScenario(SimulationScenario):
    """Scenario A: Swarm Bottleneck - 狭窄走廊"""
    
    def __init__(self, num_robots: int = 5):
        super().__init__("swarm_bottleneck", num_robots)
        
        # 走廊参数
        self.corridor_length = 18.0  # -9 到 +9
        self.corridor_width = 4.0    # -2 到 +2
        self.entrance_x = -8.0
        self.exit_x = 8.0
        
        self._generate_configs()
    
    def _generate_configs(self):
        """生成机器人的起始和目标位置"""
        import random
        random.seed(42)  # 固定种子以便复现
        
        # 起始区域：入口附近随机分布（左侧）
        start_x_min, start_x_max = -8.0, -5.0
        start_y_min, start_y_max = -1.5, 1.5
        
        # 目标区域：出口附近随机分布（右侧）
        goal_x_min, goal_x_max = 5.0, 8.0
        goal_y_min, goal_y_max = -1.5, 1.5
        
        for i in range(self.num_robots):
            # 起始位置（略微错开，避免完全重叠）
            sx = start_x_min + random.random() * (start_x_max - start_x_min)
            sy = start_y_min + random.random() * (start_y_max - start_y_min)
            syaw = random.uniform(-0.2, 0.2)  # 小角度偏航
            
            # 目标位置
            gx = goal_x_min + random.random() * (goal_x_max - goal_x_min)
            gy = goal_y_min + random.random() * (goal_y_max - goal_y_min)
            
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))
    
    def get_world_file(self) -> str:
        return 'swarm_bottleneck.sdf'


class CrossingPathsScenario(SimulationScenario):
    """Scenario B: Crossing Paths - 交叉路径 X-pattern"""
    
    def __init__(self, num_robots: int = 4):
        super().__init__("swarm_crossing_paths", num_robots)
        
        # 4个机器人：两对从对侧出发
        self.room_half_size = 10.0
        
        self._generate_configs()
    
    def _generate_configs(self):
        """两对机器人从四个角出发，交叉到对侧"""
        import random
        
        # 四个起始角点
        corners = [
            (-8, 8),    # 左上
            (8, 8),     # 右上
            (-8, -8),   # 左下
            (8, -8),    # 右下
        ]
        
        # 对应目标（对侧）
        opposite_corners = [
            (8, -8),    # 右上 -> 左下
            (-8, -8),   # 左上 -> 右下
            (8, 8),     # 左下 -> 右上
            (-8, 8),    # 右下 -> 左上
        ]
        
        for i in range(min(self.num_robots, 4)):
            sx, sy = corners[i]
            gx, gy = opposite_corners[i]
            
            # 计算初始朝向（指向大致目标方向）
            import math
            syaw = math.atan2(gy - sy, gx - sx)
            
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))
        
        # 如果 num_robots > 4，额外机器人在中心附近随机放置
        for i in range(4, self.num_robots):
            sx = random.uniform(-5, 5)
            sy = random.uniform(-5, 5)
            gx = random.uniform(-8, 8)
            gy = random.uniform(-8, 8)
            syaw = random.uniform(-math.pi, math.pi)
            
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))
    
    def get_world_file(self) -> str:
        return 'swarm_crossing_paths.sdf'


class ExpansionScenario(SimulationScenario):
    """Scenario C: Swarm Expansion - 从中心锚点向外扩展"""
    
    def __init__(self, num_robots: int = 10):
        super().__init__("swarm_expansion", num_robots)
        
        self.anchor_x, self.anchor_y = 0.0, 0.0
        self.max_radius = 12.0
        self.min_radius = 3.0
        
        self._generate_configs()
    
    def _generate_configs(self):
        """机器人从中心锚点附近出发，分散到不同方向"""
        import random
        import math
        
        for i in range(self.num_robots):
            # 均匀分布在圆周上，加入少量随机偏移
            base_angle = (2 * math.pi / self.num_robots) * i
            angle = base_angle + random.uniform(-0.2, 0.2)  # 随机扰动
            
            # 起始位置：在中心附近（锚点周围）
            start_radius = random.uniform(0.5, 1.5)
            sx = self.anchor_x + start_radius * math.cos(angle)
            sy = self.anchor_y + start_radius * math.sin(angle)
            syaw = angle  # 朝向目标方向
            
            # 目标位置：在圆周边缘（有障碍物的区域）
            goal_radius = random.uniform(self.min_radius, self.max_radius)
            gx = self.anchor_x + goal_radius * math.cos(angle)
            gy = self.anchor_y + goal_radius * math.sin(angle)
            
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))
    
    def get_world_file(self) -> str:
        return 'swarm_expansion.sdf'


class SwarmExperimentRunner:
    """
    运行 swarm 实验的主控制器
    启动仿真环境、机器人节点、收集数据
    """
    
    def __init__(self, scenario: SimulationScenario, mode: str = 'cooperative'):
        self.scenario = scenario
        self.mode = mode  # 'baseline' 或 'cooperative'
        self.logger = rclpy.logging.get_logger('swarm_experiment')
        
        # 数据记录器
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_base = f"/home/jetson/swarm_experiments"
        self.data_logger = SimulationDataLogger(log_base, scenario.name, mode)
        
        # ROS 2 节点（用于监控 metrics）
        rclpy.init(args=None)
        self.monitor_node = Node(f'experiment_monitor_{timestamp}')
        
        # 订阅 swarm metrics
        self.metrics_sub = self.monitor_node.create_subscription(
            SwarmMetrics,
            '/swarm_metrics',
            self.metrics_callback,
            10
        )
        
        # 存储 collected data
        self.collected_metrics = []
        self.robot_finished = {}
        self.experiment_complete = False
        
        # 进程管理
        self.launched_processes = []
    
    def metrics_callback(self, msg: SwarmMetrics):
        """接收并记录 metrics"""
        self.collected_metrics.append(msg)
        
        # 记录到 CSV
        self.data_logger.log_swarm_metrics(
            time.time(),
            msg
        )
        
        # 检查是否所有机器人到达目标
        if msg.all_robots_at_goal:
            self.experiment_complete = True
            self.logger.info("所有机器人已到达目标，实验结束")
    
    def generate_launch_files(self):
        """为当前场景和模式生成 launch 文件"""
        launch_dir = Path('/home/jetson/ros2_ws/src/tethered_navigation/launch/swarm_scenarios')
        launch_dir.mkdir(parents=True, exist_ok=True)
        
        launch_filename = f'{self.scenario.name}_{self.mode}.launch.py'
        launch_path = launch_dir / launch_filename
        
        self._write_launch_file(launch_path)
        return str(launch_path)
    
    def _write_launch_file(self, path: Path):
        """生成 launch 脚本"""
        
        content = f'''"""
Auto-generated launch file for {self.scenario.name} - {self.mode} mode
Generated by SwarmExperimentRunner
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'tethered_navigation'
    )
    
    world_file = os.path.join(pkg_share, 'worlds', '{self.scenario.get_world_file()}')
    
    robots = []
    
    # 启动每个机器人
    for i in range({self.scenario.num_robots}):
        robot_id = f"robot_{i+1}"
        namespace = f"/robot_{i+1}"
        
        # 起始位置和目标
        start_x, start_y, start_yaw = {self.scenario.robot_start_poses}[{i}]
        goal_x, goal_y = {self.scenario.robot_goals}[{i}]
        
        # Navigator node (C++)
        navigator = Node(
            package='tethered_navigation',
            executable='navigator_node',
            name='navigator',
            namespace=namespace,
            output='screen',
            parameters=[
                {{'use_sim_time': True}},
                {{'initial_pose_x': start_x}},
                {{'initial_pose_y': start_y}},
                {{'initial_pose_yaw': start_yaw}},
                # 设置目标点（通过 plan topic 或 parameter）
            ]
        )
        
        # Fuzzy Controller
        fuzzy_controller = Node(
            package='tethered_navigation',
            executable='fuzzy_controller.py',
            name='fuzzy_controller',
            namespace=namespace,
            output='screen',
        )
        
        # Swarm Coordinator (仅 cooperative 模式)
        swarm_nodes = []
        if "{self.mode}" == "cooperative":
            coordinator = Node(
                package='tethered_navigation',
                executable='swarm_coordinator.py',
                name='swarm_coordinator',
                namespace=namespace,
                output='screen',
                arguments=[robot_id],
            )
            broadcaster = Node(
                package='tethered_navigation',
                executable='tether_state_broadcaster.py',
                name='tether_broadcaster',
                namespace=namespace,
                output='screen',
                arguments=[robot_id],
            )
            swarm_nodes = [coordinator, broadcaster]
        
        # 合并所有节点
        robots.extend([navigator, fuzzy_controller] + swarm_nodes)
    
    # 数据记录器（仅在 robot_1 运行一次）
    data_logger = Node(
        package='tethered_navigation',
        executable='experiment_data_logger.py',  # 需要创建
        name='data_logger',
        output='screen',
        arguments=['{self.scenario.name}', '{self.mode}'],
    )
    
    return LaunchDescription(
        [data_logger] + robots
    )
'''
        
        with open(path, 'w') as f:
            f.write(content)
        
        self.logger.info(f"Launch file generated: {path}")
    
    def run_experiment(self, duration: float = 300.0):
        """
        运行单次实验
        
        Args:
            duration: 最大实验时长（秒）
        """
        self.logger.info(f"开始实验: {self.scenario.name} - {self.mode} 模式")
        self.logger.info(f"机器人数量: {self.scenario.num_robots}")
        
        # 1. 生成 launch 文件
        launch_file = self.generate_launch_files()
        
        # 2. 启动仿真（需要先启动 Gazebo 或仿真器）
        # TODO: 实现仿真器启动
        
        # 3. 启动机器人节点
        # TODO: 使用 subprocess 或 launch 系统
        
        # 4. 监控直到完成或超时
        start_time = time.time()
        
        try:
            while not self.experiment_complete and (time.time() - start_time) < duration:
                rclpy.spin_once(self.monitor_node, timeout_sec=0.1)
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.logger.warning("实验被用户中断")
        finally:
            # 5. 保存数据
            self.data_logger.end_experiment()
            
            # 6. 清理进程
            self.shutdown()
        
        return self.collected_metrics
    
    def shutdown(self):
        """清理资源"""
        for proc in self.launched_processes:
            proc.terminate()
        
        self.monitor_node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Run swarm robotics experiments')
    parser.add_argument('--scenario', type=str, required=True,
                        choices=['bottleneck', 'crossing', 'expansion'],
                        help='Scenario to run')
    parser.add_argument('--mode', type=str, required=True,
                        choices=['baseline', 'cooperative'],
                        help='Experiment mode')
    parser.add_argument('--num-robots', type=int, default=5,
                        help='Number of robots')
    parser.add_argument('--duration', type=float, default=300.0,
                        help='Maximum experiment duration (seconds)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed')
    
    args = parser.parse_args()
    
    # 选择场景
    if args.scenario == 'bottleneck':
        scenario = BottleneckScenario(args.num_robots)
    elif args.scenario == 'crossing':
        scenario = CrossingPathsScenario(args.num_robots)
    elif args.scenario == 'expansion':
        scenario = ExpansionScenario(args.num_robots)
    else:
        raise ValueError(f"Unknown scenario: {args.scenario}")
    
    runner = SwarmExperimentRunner(scenario, args.mode)
    
    try:
        metrics = runner.run_experiment(duration=args.duration)
        
        # 打印摘要
        print("\n" + "="*60)
        print("实验完成！")
        print(f"场景: {scenario.name}")
        print(f"模式: {args.mode}")
        print(f"机器人数量: {args.num_robots}")
        print(f"收集的 metrics 数量: {len(metrics)}")
        
        if metrics:
            last = metrics[-1]
            print(f"最终纠缠次数: {last.total_entanglements}")
            print(f"最终平均张力: {last.avg_tension:.2f} N")
            print(f"全部到达目标: {last.all_robots_at_goal}")
        
        print("="*60)
        
    except Exception as e:
        print(f"实验失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        runner.shutdown()


if __name__ == '__main__':
    main()
