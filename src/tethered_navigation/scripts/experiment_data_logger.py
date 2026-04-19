#!/usr/bin/env python3
"""
Experiment Data Logger for Swarm Robotics
==========================================
收集并保存所有机器人的实验数据：
- 轨迹
- 张力历史
- Yielding 决策
- 交叉事件
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import csv
import json
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional

# ROS 2 Messages
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tethered_navigation.msg import TetherState, YieldingDecision, SwarmMetrics


class ExperimentDataLogger(Node):
    """
    集中式数据记录器
    收集所有机器人的状态并保存为 CSV/JSON
    """
    
    def __init__(self, scenario_name: str, mode: str, num_robots: int):
        super().__init__('experiment_data_logger')
        
        self.scenario_name = scenario_name
        self.mode = mode
        self.num_robots = num_robots
        self.start_time = time.time()
        
        # 创建日志目录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_base = Path(f"/home/jetson/swarm_experiments/{timestamp}_{scenario_name}_{mode}")
        self.log_base.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f"日志目录: {self.log_base}")
        
        # --- 数据文件 ---
        # 1. 每个机器人的轨迹
        self.traj_files: Dict[str, csv.writer] = {}
        self.traj_handles: Dict[str, any] = {}
        
        # 2. 交叉事件日志
        self.crossing_file = open(self.log_base / 'crossing_events.csv', 'w', newline='')
        self.crossing_writer = csv.writer(self.crossing_file)
        self.crossing_writer.writerow([
            'timestamp', 'robot_a', 'robot_b',
            'crossing_x', 'crossing_y', 'crossing_z',
            'severity', 'action_taken'
        ])
        
        # 3. Swarm 指标
        self.swarm_file = open(self.log_base / 'swarm_metrics.csv', 'w', newline='')
        self.swarm_writer = csv.writer(self.swarm_file)
        self.swarm_writer.writerow([
            'timestamp', 'active_robots', 'total_entanglements',
            'resolved_entanglements', 'avg_tension', 'max_tension',
            'swarm_traversal_time', 'all_at_goal'
        ])
        
        # 4. Yielding 决策日志
        self.yielding_file = open(self.log_base / 'yielding_decisions.csv', 'w', newline='')
        self.yielding_writer = csv.writer(self.yielding_file)
        self.yielding_writer.writerow([
            'timestamp', 'requester', 'target',
            'crossing_x', 'crossing_y', 'severity',
            'suggested_action', 'yield_duration_est'
        ])
        
        # 5. 实验元数据
        self.metadata = {
            'scenario': scenario_name,
            'mode': mode,
            'num_robots': num_robots,
            'start_time': datetime.now().isoformat(),
            'seed': 42,
            'description': ''
        }
        
        # --- 订阅者 ---
        # QoS - 可靠传输
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # 订阅所有机器人的 tether state
        self.tether_subscriptions: Dict[str, any] = {}
        
        # 订阅 swarm metrics
        self.swarm_sub = self.create_subscription(
            SwarmMetrics,
            '/swarm_metrics',
            self.swarm_callback,
            qos_reliable
        )
        
        # 订阅 yielding decisions
        self.yielding_sub = self.create_subscription(
            YieldingDecision,
            '/yielding_decisions',
            self.yielding_callback,
            qos_reliable
        )
        
        # 为每个机器人创建轨迹订阅者
        for i in range(num_robots):
            robot_id = f"robot_{i+1}"
            namespace = f"/{robot_id}"
            
            # Odometry 订阅
            odom_sub = self.create_subscription(
                Odometry,
                f'{namespace}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )
            self.tether_subscriptions[robot_id] = odom_sub
            
            # 创建该机器人的轨迹文件
            traj_file = open(self.log_base / f'trajectory_{robot_id}.csv', 'w', newline='')
            traj_writer = csv.writer(traj_file)
            traj_writer.writerow([
                'timestamp', 'x', 'y', 'theta',
                'linear_x', 'angular_z',
                'tension', 'tether_length',
                'yielding_action'
            ])
            self.traj_files[robot_id] = traj_writer
            self.traj_handles[robot_id] = traj_file
        
        # 定时保存元数据
        self.save_timer = self.create_timer(30.0, self.save_metadata_periodic)
        
        self.get_logger().info("Experiment Data Logger 初始化完成")
    
    def odom_callback(self, msg: Odometry, robot_id: str):
        """记录机器人位姿"""
        rel_time = time.time() - self.start_time
        
        # 从其他 topic 获取 tether 状态和 yielding action
        # 这里我们先记录基本位姿，后续需要补充
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # 简单的四元数转 yaw
        import math
        yaw = 2.0 * math.atan2(orient.z, orient.w)
        
        # 线速度和角速度
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        
        # 写入轨迹文件（需要后续补充 tension 和 yielding_action）
        # 暂时先写占位符
        self.traj_files[robot_id].writerow([
            rel_time,
            pos.x, pos.y, yaw,
            linear_x, angular_z,
            -1.0,  # tension (待补充)
            -1.0,  # tether_length (待补充)
            'unknown'  # yielding_action (待补充)
        ])
    
    def swarm_callback(self, msg: SwarmMetrics):
        """记录 swarm 级别指标"""
        rel_time = time.time() - self.start_time
        self.swarm_writer.writerow([
            rel_time,
            msg.active_robots,
            msg.total_entanglements,
            msg.resolved_entanglements,
            msg.avg_tension,
            msg.max_tension,
            msg.swarm_traversal_time,
            msg.all_robots_at_goal
        ])
        
        # 检查实验是否完成
        if msg.all_robots_at_goal:
            self.get_logger().info("检测到所有机器人到达目标，准备结束实验")
            self.end_experiment()
    
    def yielding_callback(self, msg: YieldingDecision):
        """记录 yielding 决策"""
        rel_time = time.time() - self.start_time
        self.yielding_writer.writerow([
            rel_time,
            msg.requesting_robot_id,
            msg.target_robot_id,
            msg.crossing_point.x,
            msg.crossing_point.y,
            msg.crossing_point.z,
            msg.crossing_severity,
            msg.suggested_action
        ])
    
    def save_metadata_periodic(self):
        """定期保存元数据（以便崩溃恢复）"""
        try:
            with open(self.log_base / 'metadata.json', 'w') as f:
                json.dump(self.metadata, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f"保存元数据失败: {e}")
    
    def end_experiment(self):
        """结束实验，保存所有数据"""
        self.get_logger().info("正在结束实验，保存数据...")
        
        end_time = time.time()
        self.metadata['end_time'] = datetime.now().isoformat()
        self.metadata['total_duration'] = end_time - self.start_time
        
        # 保存最终元数据
        with open(self.log_base / 'metadata.json', 'w') as f:
            json.dump(self.metadata, f, indent=2)
        
        # 关闭所有文件
        for handle in self.traj_handles.values():
            handle.close()
        
        self.crossing_file.close()
        self.swarm_file.close()
        self.yielding_file.close()
        
        self.get_logger().info(f"所有数据已保存到: {self.log_base}")
        
        # 可选：生成摘要
        self.generate_summary()
    
    def generate_summary(self):
        """生成实验摘要（文本格式）"""
        summary_path = self.log_base / 'summary.txt'
        
        # 读取数据并生成统计
        summary_lines = [
            "=" * 60,
            f"实验: {self.scenario_name} - {self.mode}",
            f"机器人数量: {self.num_robots}",
            f"开始时间: {self.metadata['start_time']}",
            f"总时长: {self.metadata.get('total_duration', 0):.1f} 秒",
            "",
            "数据文件:",
            f"  - 轨迹: trajectory_robot_*.csv (每个机器人一个)",
            f"  - Swarm Metrics: swarm_metrics.csv",
            f"  - 交叉事件: crossing_events.csv",
            f"  - 决策记录: yielding_decisions.csv",
            "",
            "下一步: 使用 analyze_results.py 生成图表和统计",
            "=" * 60
        ]
        
        with open(summary_path, 'w') as f:
            f.write('\n'.join(summary_lines))
        
        self.get_logger().info(f"摘要已保存: {summary_path}")


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    if len(sys.argv) < 3:
        print("用法: experiment_data_logger.py <scenario_name> <mode> <num_robots>")
        sys.exit(1)
    
    scenario_name = sys.argv[1]
    mode = sys.argv[2]
    num_robots = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    
    logger = ExperimentDataLogger(scenario_name, mode, num_robots)
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info("接收到中断信号，正在保存数据...")
        logger.end_experiment()
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
