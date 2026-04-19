#!/usr/bin/env python3
"""
Swarm Coordinator Node for Tethered Robots
==========================================
实现了多机器人协同逻辑：检测 tether 交叉、基于优先级决策、yield/reroute 策略

运行在每个机器人上，负责：
1. 监听所有其他机器人的 tether 状态 (/swarm_tether_states)
2. 检测本机 tether 与其他机器人 tether 的交叉
3. 基于优先级规则决定谁应该让行
4. 发布 yielding 决策和接收其他机器人的决策
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

# ROS 2 Messages
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Point, Twist
from tethered_navigation.msg import TetherState, YieldingDecision, SwarmMetrics

# 导入 fuzzy controller（如果可用）
try:
    from tethered_navigation.fuzzy_controller import FuzzyControllerNode
except ImportError:
    FuzzyControllerNode = None


class SwarmCoordinator(Node):
    """
    分布式 swarm 协调器
    
    每个机器人实例运行此节点，实现：
    - 多机器人 tether 交叉检测
    - 基于优先级的 yield 决策
    - 协同避障行为
    """
    
    def __init__(self, robot_id: str, robot_namespace: str = '/robot_1'):
        super().__init__(f'swarm_coordinator_{robot_id}')
        
        self.robot_id = robot_id
        self.robot_namespace = robot_namespace
        
        # QoS 配置 - 需要可靠传输以进行协调
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        
        # --- Swarm 状态存储 ---
        self.swarm_tether_states: Dict[str, TetherState] = {}  # robot_id -> TetherState
        self.tether_segments_cache: Dict[str, List[Tuple[Point, Point]]] = {}  # 缓存 tether 线段
        
        # --- 优先级系统 ---
        # 优先级规则（可配置）：
        # 1. 离目标越近，优先级越高（避免远处机器人阻挡近处机器人）
        # 2. 张力越大，优先级越高（高张力机器人需要更快通过）
        # 3. 可配置的静态优先级（手动分配）
        self.static_priority = self.get_parameter('static_priority').value  # 0.0-1.0
        self.goal_position = None  # 需要在运行时设置
        self.priority_weights = {
            'distance_to_goal': 0.5,  # 距离目标的距离（越近越高）
            'tension': 0.3,           # tether 张力（越高越紧急）
            'static': 0.2,            # 静态优先级
        }
        
        # --- 交叉检测参数 ---
        self.crossing_detection_threshold = 0.15  # 线段相交距离阈值 (m)
        self.tether_segment_spacing = 0.2  # tether 采样间隔 (m)
        
        # --- Yield 决策 ---
        self.yielding_actions = ['maintain', 'yield', 'reroute', 'speed_up']
        self.current_yielding_action = 'maintain'
        self.yield_cooldown = 5.0  # 让行后冷却时间 (s)
        self.last_yield_time = 0.0
        
        # --- 指标统计 ---
        self.metrics = {
            'crossings_detected': 0,
            'yields_executed': 0,
            'entanglements_avoided': 0,
            'total_decision_time': 0.0,
            'decision_count': 0,
        }
        
        # --- 订阅者 ---
        self.swarm_sub = self.create_subscription(
            TetherState,
            '/swarm_tether_states',
            self.swarm_tether_callback,
            qos_reliable
        )
        
        # --- 发布者 ---
        self.yielding_pub = self.create_publisher(
            YieldingDecision,
            '/yielding_decisions',
            qos_reliable
        )
        
        self.metrics_pub = self.create_publisher(
            SwarmMetrics,
            '/swarm_metrics',
            10
        )
        
        # --- 可选：集成 Fuzzy Controller ---
        self.fuzzy_input_pub = None
        if FuzzyControllerNode:
            self.fuzzy_input_pub = self.create_publisher(
                Float32MultiArray,
                f'{self.robot_namespace}/fuzzy_input',
                10
            )
        
        # 定时器
        self.decision_timer = self.create_timer(0.1, self.decision_loop)  # 10 Hz
        self.metrics_timer = self.create_timer(1.0, self.publish_metrics)  # 1 Hz
        
        self.get_logger().info(f'SwarmCoordinator for {robot_id} initialized')
    
    def set_goal(self, goal_pose: PoseStamped):
        """设置目标位置，用于优先级计算"""
        self.goal_position = goal_pose.pose.position
    
    def swarm_tether_callback(self, msg: TetherState):
        """接收其他机器人的 tether 状态"""
        self.swarm_tether_states[msg.robot_id] = msg
        
        # 更新缓存：将 tether_segment_points 转换为线段以便交叉检测
        if msg.tether_segment_points and len(msg.tether_segment_points) >= 2:
            segments = []
            points = msg.tether_segment_points
            for i in range(len(points) - 1):
                segments.append((points[i], points[i+1]))
            self.tether_segments_cache[msg.robot_id] = segments
    
    def compute_priority(self, robot_id: str, tether_state: Optional[TetherState] = None) -> float:
        """
        计算机器人的优先级得分 (0.0 - 1.0)
        高优先级机器人应该优先通过，低优先级机器人应该 yield
        """
        if robot_id == self.robot_id:
            # 自己的优先级基于静态配置
            base_priority = self.static_priority
        else:
            # 其他机器人：从消息中提取或使用默认值
            if tether_state and tether_state.priority_score > 0:
                base_priority = tether_state.priority_score
            else:
                base_priority = 0.5  # 默认中等优先级
        
        # TODO: 可以根据动态因素调整（距离目标、张力等）
        return base_priority
    
    def detect_crossing(self, my_segments: List[Tuple[Point, Point]], 
                       other_segments: List[Tuple[Point, Point]], 
                       other_robot_id: str) -> Optional[Tuple[Point, float]]:
        """
        检测本机 tether 与另一机器人 tether 的交叉
        
        Returns:
            (crossing_point, severity) 如果检测到交叉
            None 如果没有交叉
        """
        if not my_segments or not other_segments:
            return None
        
        # 简单的线段相交检测
        for seg1 in my_segments:
            for seg2 in other_segments:
                crossing = self.line_segments_intersect(seg1, seg2)
                if crossing:
                    # 计算交叉严重程度（张力乘积）
                    my_tension = 30.0  # TODO: 从实际状态获取
                    other_tension = 30.0
                    if other_robot_id in self.swarm_tether_states:
                        other_tension = self.swarm_tether_states[other_robot_id].tension
                    
                    severity = min(1.0, (my_tension * other_tension) / 2500.0)
                    return crossing, severity
        
        return None
    
    def line_segments_intersect(self, seg1: Tuple[Point, Point], 
                               seg2: Tuple[Point, Point]) -> Optional[Point]:
        """
        检测两条线段是否相交，返回交叉点
        使用二维投影（忽略 z 坐标）
        """
        p1, p2 = seg1
        p3, p4 = seg2
        
        # 转换为 numpy 数组（仅 x, y）
        def to_vec(p):
            return np.array([p.x, p.y])
        
        a = to_vec(p1)
        b = to_vec(p2)
        c = to_vec(p3)
        d = to_vec(p4)
        
        # 线段相交检测（二维）
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
        
        intersect = ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)
        
        if intersect:
            # 计算交点（简化版）
            return Point(x=(a[0]+b[0])/2, y=(a[1]+b[1])/2, z=0.0)
        return None
    
    def make_yielding_decision(self, other_robot_id: str, crossing_point: Point, 
                              severity: float) -> YieldingDecision:
        """
        基于优先级做出 yield/reroute 决策
        """
        my_priority = self.compute_priority(self.robot_id)
        other_priority = self.compute_priority(other_robot_id, 
                                               self.swarm_tether_states.get(other_robot_id))
        
        decision = YieldingDecision()
        decision.header = Header(stamp=self.get_clock().now().to_msg())
        decision.requesting_robot_id = self.robot_id
        decision.target_robot_id = other_robot_id
        decision.crossing_point = crossing_point
        decision.crossing_severity = severity
        
        # 决策逻辑：
        # - 如果我的优先级显著低于对方（< 0.8倍），我应该 yield
        # - 如果我的张力远高于对方，我应该优先通过（即使优先级略低）
        # - 如果交叉严重性很高，双方都应该调整
        
        priority_ratio = my_priority / (other_priority + 1e-6)
        
        if priority_ratio < 0.7:
            # 我的优先级低，我让行
            decision.yield_requested = True
            decision.suggested_action = 'yield'
            decision.priority_comparison = priority_ratio
        elif priority_ratio > 1.3:
            # 我的优先级高，请求对方让行
            decision.yield_requested = False  # 不请求自己让行，但暗示对方应该
            decision.suggested_action = 'reroute'  # 建议对方 reroute
            decision.priority_comparison = priority_ratio
        else:
            # 优先级相近，协商解决
            decision.suggested_action = 'maintain'
            decision.priority_comparison = 1.0
        
        # 估算让行时间（基于距离和速度）
        decision.yield_duration_est = 2.0  # 默认 2 秒
        
        return decision
    
    def decision_loop(self):
        """主决策循环（10 Hz）"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        if not self.tether_segments_cache:
            return  # 还没有收到任何 tether 数据
        
        # 获取自己的 tether segments（需要从 navigator 或其他节点获取）
        my_segments = self.get_my_tether_segments()
        if not my_segments:
            return
        
        # 检测与每个其他机器人的交叉
        for other_robot_id, other_segments in self.tether_segments_cache.items():
            if other_robot_id == self.robot_id:
                continue
            
            crossing = self.detect_crossing(my_segments, other_segments, other_robot_id)
            if crossing:
                crossing_point, severity = crossing
                self.metrics['crossings_detected'] += 1
                
                # 做出决策
                decision = self.make_yielding_decision(other_robot_id, crossing_point, severity)
                self.yielding_pub.publish(decision)
                
                # 更新本地动作
                if decision.suggested_action == 'yield':
                    self.current_yielding_action = 'yield'
                    self.last_yield_time = current_time
                    self.metrics['yields_executed'] += 1
                elif decision.suggested_action == 'reroute':
                    self.current_yielding_action = 'reroute'
                
                self.get_logger().debug(
                    f'Crossing with {other_robot_id}: action={decision.suggested_action}, '
                    f'severity={severity:.2f}'
                )
        
        # 冷却检查
        if current_time - self.last_yield_time > self.yield_cooldown:
            self.current_yielding_action = 'maintain'
    
    def get_my_tether_segments(self) -> List[Tuple[Point, Point]]:
        """
        获取本机 tether 的线段表示
        需要从 navigator 或 tether 管理器获取实际数据
        """
        # TODO: 从实际 tether 状态构建
        # 现在返回空，需要集成到系统中
        return []
    
    def publish_metrics(self):
        """发布 swarm 级别指标"""
        metrics_msg = SwarmMetrics()
        metrics_msg.header = Header(stamp=self.get_clock().now().to_msg())
        metrics_msg.active_robots = len(self.swarm_tether_states) + 1
        metrics_msg.total_entanglements = self.metrics['crossings_detected']
        metrics_msg.resolved_entanglements = self.metrics['yields_executed']
        metrics_msg.avg_tension = 20.0  # TODO: 计算实际平均值
        metrics_msg.swarm_traversal_time = 0.0  # TODO: 跟踪时间
        metrics_msg.all_robots_at_goal = False
        
        self.metrics_pub.publish(metrics_msg)
    
    def get_current_action(self) -> str:
        """供 fuzzy controller 查询的当前协同动作"""
        return self.current_yielding_action


def main(args=None):
    rclpy.init(args=args)
    
    # 从参数服务器获取 robot_id（或在命名空间中推断）
    # 例如: /robot_1 -> robot_id = "robot_1"
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "robot_1"
    namespace = f'/{robot_id}'
    
    coordinator = SwarmCoordinator(robot_id, namespace)
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
