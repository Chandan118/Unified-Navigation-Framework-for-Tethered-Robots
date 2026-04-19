#!/usr/bin/env python3
"""
Tether State Broadcaster for Tethered Robots
=============================================
每个机器人运行此节点，定期发布其 tether 状态到 /swarm_tether_states

功能：
1. 从 navigator 获取当前位置和 tether 张力
2. 计算 tether 的关键采样点（用于交叉检测）
3. 计算优先级得分
4. 广播到 swarm 网络
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import List, Tuple
from collections import deque

# ROS 2 Messages
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry
from tethered_navigation.msg import TetherState


class TetherStateBroadcaster(Node):
    """
    发布本机 tether 状态供 swarm 协调使用
    """
    
    def __init__(self, robot_id: str, robot_namespace: str = '/robot_1'):
        super().__init__(f'tether_broadcaster_{robot_id}')
        
        self.robot_id = robot_id
        self.robot_namespace = robot_namespace
        
        # QoS - 可靠传输
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- 状态变量 ---
        self.current_pose = None  # 从 odom 获取
        self.tether_tension = 0.0  # 从 navigator/fuzzy controller 获取
        self.tether_anchor_point = Point(x=0.0, y=0.0, z=0.0)  # 锚点（通常为出发点）
        self.tether_length = 0.0  # 当前 tether 长度
        self.max_tether_length = 10.0  # 最大 tether 长度 (m)
        
        # --- Tether 采样配置 ---
        self.tether_sampling_distance = 0.2  # 每 20cm 一个采样点
        self.tether_history = deque(maxlen=50)  # 轨迹历史（用于可视化）
        
        # --- 优先级计算 ---
        self.goal_position = None
        self.static_priority = 0.5  # 默认优先级
        
        # --- 订阅者 ---
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.robot_namespace}/odom',
            self.odom_callback,
            10
        )
        
        # 订阅 tether 张力（从 navigator 或 fuzzy controller）
        # 假设 navigator 发布 tether 状态到某个 topic
        self.tension_sub = self.create_subscription(
            Float32MultiArray,
            f'{self.robot_namespace}/tether_status',  # 需要定义
            self.tension_callback,
            10
        )
        
        # --- 发布者 ---
        self.tether_state_pub = self.create_publisher(
            TetherState,
            '/swarm_tether_states',
            qos_reliable
        )
        
        # --- 定时器 ---
        self.broadcast_timer = self.create_timer(0.1, self.broadcast_tether_state)  # 10 Hz
        
        self.get_logger().info(f'TetherStateBroadcaster for {robot_id} started')
    
    def odom_callback(self, msg: Odometry):
        """接收里程计信息"""
        self.current_pose = msg.pose.pose
        # 记录轨迹历史
        self.tether_history.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ))
    
    def tension_callback(self, msg: Float32MultiArray):
        """接收 tether 状态（张力、长度等）"""
        if len(msg.data) >= 2:
            self.tether_tension = msg.data[0]  # 张力 (N)
            self.tether_length = msg.data[1]   # 长度 (m)
            # msg.data[2] 可能包含 slack 等
    
    def set_goal(self, goal_pose: PoseStamped):
        """设置目标位置"""
        self.goal_position = goal_pose.pose.position
    
    def compute_priority(self) -> float:
        """
        计算机器人的优先级得分
        优先级综合：
        - 静态优先级（手动分配）
        - 距离目标的远近（越近越高）
        - Tether 张力（张力越大越紧急）
        """
        priority = self.static_priority
        
        if self.goal_position and self.current_pose:
            # 距离归一化（假设最大感知距离 20m）
            dist = np.sqrt(
                (self.current_pose.position.x - self.goal_position.x) ** 2 +
                (self.current_pose.position.y - self.goal_position.y) ** 2
            )
            dist_factor = max(0.0, 1.0 - dist / 20.0)
            priority += self.priority_weights['distance_to_goal'] * dist_factor
        
        # 张力因子（高张力需要优先处理）
        tension_factor = min(1.0, self.tether_tension / 50.0)
        priority += self.priority_weights['tension'] * tension_factor
        
        return np.clip(priority, 0.0, 1.0)
    
    def compute_tether_segments(self) -> List[Tuple[Point, Point]]:
        """
        计算 tether 的线段表示
        从锚点到当前位置，按固定间隔采样
        """
        if not self.current_pose:
            return []
        
        segments = []
        
        # 锚点（通常为起始位置）
        anchor = self.tether_anchor_point
        
        # 当前机器人位置
        current_pos = self.current_pose.position
        
        # 计算 tether 方向向量
        direction = np.array([
            current_pos.x - anchor.x,
            current_pos.y - anchor.y,
            current_pos.z - anchor.z
        ])
        
        tether_dist = np.linalg.norm(direction)
        if tether_dist < 1e-6:
            return []
        
        direction = direction / tether_dist
        
        # 沿 tether 采样关键点
        num_samples = max(2, int(tether_dist / self.tether_sampling_distance))
        
        points = []
        for i in range(num_samples):
            t = i / (num_samples - 1)
            px = anchor.x + direction[0] * tether_dist * t
            py = anchor.y + direction[1] * tether_dist * t
            pz = anchor.z + direction[2] * tether_dist * t
            points.append(Point(x=px, y=py, z=pz))
        
        # 生成线段
        for i in range(len(points) - 1):
            segments.append((points[i], points[i+1]))
        
        return segments
    
    def check_tether_crossing(self, segments: List[Tuple[Point, Point]]) -> bool:
        """
        快速检测 tether 是否可能与其他机器人交叉
        通过检查历史轨迹的凸包或包围盒
        """
        if len(segments) < 2:
            return False
        
        # 简单的包围盒检查
        xs = [p.x for seg in segments for p in seg]
        ys = [p.y for seg in segments for p in seg]
        
        # 如果 tether 包围盒太小，不太可能交叉
        if max(xs) - min(xs) < 0.1 and max(ys) - min(ys) < 0.1:
            return False
        
        return True
    
    def broadcast_tether_state(self):
        """发布 tether 状态"""
        if not self.current_pose:
            return
        
        msg = TetherState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.robot_id = self.robot_id
        
        # 锚点位置（需要从起始点记录）
        msg.anchor_pose = PoseStamped()
        msg.anchor_pose.header = msg.header
        msg.anchor_pose.pose.position = self.tether_anchor_point
        
        msg.tension = float(self.tether_tension)
        msg.slack_length = max(0.0, self.max_tether_length - self.tether_length)
        msg.current_length = float(self.tether_length)
        
        # 计算并添加 tether 线段（用于交叉检测）
        segments = self.compute_tether_segments()
        msg.tether_segment_points = []
        for seg in segments:
            msg.tether_segment_points.append(seg[0])
        if segments:
            msg.tether_segment_points.append(segments[-1][1])
        
        # 交叉标记（由 swarm coordinator 更新，这里先设为 false）
        msg.is_crossing = self.check_tether_crossing(segments)
        
        # 优先级
        msg.priority_score = float(self.compute_priority())
        
        self.tether_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "robot_1"
    namespace = f'/{robot_id}'
    
    broadcaster = TetherStateBroadcaster(robot_id, namespace)
    
    try:
        rclpy.spin(broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
