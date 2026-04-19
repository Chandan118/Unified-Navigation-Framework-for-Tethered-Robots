#!/usr/bin/env python3
"""
2D Swarm Simulator for Tethered Robots
=======================================
轻量级多机器人仿真器，专为 swarm tether 协调设计

特点：
- 纯 Python 实现，无 heavy 依赖（不需要 Gazebo）
- 基于 ROS 2 话题（/odom, /scan, /cmd_vel）
- 模拟 tether 物理（张力、松弛、交叉检测）
- 支持 10+ 机器人实时仿真
- 可直接在 Jetson Orin Nano 运行
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
import numpy as np
from typing import List, Dict, Tuple, Optional
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
import json
import csv

# ROS 2 Messages
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Imu
from tethered_navigation.msg import TetherState, SwarmMetrics


@dataclass
class RobotState:
    """单个机器人的完整状态"""
    robot_id: str
    pose: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, theta
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))  # vx, vy
    tether_anchor: np.ndarray = field(default_factory=lambda: np.zeros(3))  # 锚点
    tether_length: float = 0.0
    tether_tension: float = 0.0
    max_tether_length: float = 10.0
    tether_k: float = 100.0  # 弹簧刚度
    tether_damping: float = 5.0  # 阻尼
    goal: np.ndarray = field(default_factory=lambda: np.zeros(2))
    history: deque = field(default_factory=lambda: deque(maxlen=100))
    yielding_action: str = 'maintain'
    radius: float = 0.2  # 机器人半径（用于碰撞检测）


class TetherPhysics:
    """Tether 物理模拟（弹簧-质点模型）"""
    
    @staticmethod
    def compute_tension(robot_pos: np.ndarray, anchor: np.ndarray, 
                       velocity: np.ndarray, rest_length: float,
                       k: float, damping: float) -> Tuple[float, np.ndarray]:
        """
        计算 tether 张力和力
        
        Returns:
            tension: 张力大小 (N)
            force: 作用在机器人上的力向量 (x, y)
        """
        # 向量：从锚点到机器人
        displacement = robot_pos[:2] - anchor[:2]
        current_length = np.linalg.norm(displacement)
        
        if current_length < 1e-6:
            return 0.0, np.zeros(2)
        
        # 单位方向向量
        direction = displacement / current_length
        
        # 弹簧力：F_spring = k * (current_length - rest_length)
        spring_force = k * max(0, current_length - rest_length)
        
        # 阻尼力：F_damping = damping * (v · direction)
        radial_velocity = np.dot(velocity[:2], direction)
        damping_force = damping * max(0, radial_velocity)  # 只对拉伸方向阻尼
        
        # 总张力
        tension = spring_force + damping_force
        
        # 作用力（指向锚点方向，当拉伸时）
        force = -direction * tension
        
        return tension, force


class SwarmSimulator(Node):
    """
    多机器人仿真器主节点
    模拟物理世界、机器人运动、传感器和 tether
    """
    
    def __init__(self, scenario_name: str = 'bottleneck', num_robots: int = 5):
        super().__init__('swarm_simulator')
        
        self.scenario_name = scenario_name
        self.num_robots = num_robots
        
        # 加载场景配置
        self.robots: Dict[str, RobotState] = {}
        self.obstacles: List[Dict] = []
        self.bounds = {'xmin': -15, 'xmax': 15, 'ymin': -15, 'ymax': 15}
        
        self._load_scenario(scenario_name)
        
        # 仿真参数
        self.dt = 0.02  # 50 Hz 仿真步长
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # 传感器参数
        self.lidar_range = 8.0
        self.lidar_angle_min = -np.pi
        self.lidar_angle_max = np.pi
        self.lidar_angle_increment = np.pi / 180  # 1度一个点
        self.lidar_num_readings = 360
        
        # 物理参数
        self.robot_mass = 2.0  # kg
        self.friction_coeff = 0.8
        
        # 记录交叉事件（用于数据收集）
        self.crossing_events = []
        self.entanglement_count = 0
        
        # --- ROS 2 Publishers ---
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 为每个机器人创建发布者
        self.odom_publishers: Dict[str, any] = {}
        self.scan_publishers: Dict[str, any] = {}
        self.tether_publishers: Dict[str, any] = {}
        
        for i in range(num_robots):
            robot_id = f"robot_{i+1}"
            namespace = f'/{robot_id}'
            
            # Odometry
            self.odom_publishers[robot_id] = self.create_publisher(
                Odometry, f'{namespace}/odom', qos_reliable)
            
            # LaserScan
            self.scan_publishers[robot_id] = self.create_publisher(
                LaserScan, f'{namespace}/scan', qos_reliable)
            
            # Tether State (用于 swarm coordination)
            self.tether_publishers[robot_id] = self.create_publisher(
                TetherState, '/swarm_tether_states', qos_reliable)
        
        # Swarm Metrics 发布
        self.metrics_pub = self.create_publisher(SwarmMetrics, '/swarm_metrics', 10)
        
        # --- ROS 2 Subscribers ---
        # 订阅每个机器人的 cmd_vel
        self.cmd_vel_subs: Dict[str, any] = {}
        for i in range(num_robots):
            robot_id = f"robot_{i+1}"
            namespace = f'/{robot_id}'
            sub = self.create_subscription(
                Twist,
                f'{namespace}/cmd_vel',
                lambda msg, rid=robot_id: self.cmd_vel_callback(msg, rid),
                10
            )
            self.cmd_vel_subs[robot_id] = sub
        
        # --- 定时器 ---
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)  # 50 Hz 物理
        self.publish_timer = self.create_timer(0.05, self.publish_sensor_data)  # 20 Hz 传感器
        
        self.sim_time = 0.0
        self.step_count = 0
        
        self.get_logger().info(f"Swarm Simulator 启动: {scenario_name}, {num_robots} 机器人")
    
    def _load_scenario(self, scenario_name: str):
        """加载场景配置"""
        # 重置机器人
        self.robots.clear()
        self.obstacles.clear()
        
        if scenario_name == 'bottleneck':
            # 狭窄走廊场景
            # 障碍物：左右墙壁
            self.obstacles = [
                {'type': 'wall', 'x1': -9, 'y1': -2.5, 'x2': 9, 'y2': -2.5, 'thickness': 0.2},  # 下墙
                {'type': 'wall', 'x1': -9, 'y1': 2.5, 'x2': 9, 'y2': 2.5, 'thickness': 0.2},   # 上墙
                {'type': 'wall', 'x1': -9, 'y1': -2.5, 'x2': -9, 'y2': 2.5, 'thickness': 0.2},  # 入口左
                {'type': 'wall', 'x1': 9, 'y1': -2.5, 'x2': 9, 'y2': 2.5, 'thickness': 0.2},    # 出口右
            ]
            
            # 生成机器人初始状态
            import random
            random.seed(42)
            for i in range(self.num_robots):
                robot_id = f"robot_{i+1}"
                # 起始位置（入口左侧）
                sx = -7.0 + random.uniform(-1.0, 1.0)
                sy = random.uniform(-1.0, 1.0)
                syaw = random.uniform(-0.3, 0.3)
                
                # 目标位置（出口右侧）
                gx = 7.0 + random.uniform(-1.0, 1.0)
                gy = random.uniform(-1.0, 1.0)
                
                robot = RobotState(
                    robot_id=robot_id,
                    pose=np.array([sx, sy, syaw]),
                    tether_anchor=np.array([sx, sy, 0.0]),  # 锚点在起始点
                    tether_length=0.0,
                    goal=np.array([gx, gy]),
                    radius=0.2
                )
                robot.max_tether_length = 8.0  # 根据场景调整
                self.robots[robot_id] = robot
                
        elif scenario_name == 'crossing_paths':
            # 交叉路径场景
            self.obstacles = [
                {'type': 'wall', 'x1': -12, 'y1': 12, 'x2': 12, 'y2': 12, 'thickness': 0.3},  # 上
                {'type': 'wall', 'x1': -12, 'y1': -12, 'x2': 12, 'y2': -12, 'thickness': 0.3}, # 下
                {'type': 'wall', 'x1': -12, 'y1': -12, 'x2': -12, 'y2': 12, 'thickness': 0.3}, # 左
                {'type': 'wall', 'x1': 12, 'y1': -12, 'x2': 12, 'y2': 12, 'thickness': 0.3},   # 右
                {'type': 'box', 'x': 0, 'y': 0, 'size': 2.0},  # 中心障碍物
            ]
            
            import math
            for i in range(self.num_robots):
                robot_id = f"robot_{i+1}"
                angle = (2 * math.pi / self.num_robots) * i
                radius = 9.0
                
                sx = radius * math.cos(angle)
                sy = radius * math.sin(angle)
                syaw = angle + math.pi  # 朝向中心
                gx = -sx
                gy = -sy
                
                robot = RobotState(
                    robot_id=robot_id,
                    pose=np.array([sx, sy, syaw]),
                    tether_anchor=np.array([sx, sy, 0.0]),
                    tether_length=0.0,
                    goal=np.array([gx, gy]),
                    radius=0.2
                )
                robot.max_tether_length = 20.0
                self.robots[robot_id] = robot
                
        elif scenario_name == 'expansion':
            # 扩展场景
            self.obstacles = []
            # 边界
            self.bounds = {'xmin': -15, 'xmax': 15, 'ymin': -15, 'ymax': 15}
            
            # 随机障碍物（30+个）
            import random
            random.seed(42)
            for _ in range(35):
                obs = {
                    'type': random.choice(['box', 'cylinder']),
                    'x': random.uniform(-12, 12),
                    'y': random.uniform(-12, 12),
                }
                if obs['type'] == 'box':
                    obs['size'] = random.uniform(0.3, 0.8)
                else:
                    obs['radius'] = random.uniform(0.2, 0.4)
                    obs['height'] = random.uniform(0.5, 1.0)
                self.obstacles.append(obs)
            
            # 机器人从中心锚点出发
            for i in range(self.num_robots):
                robot_id = f"robot_{i+1}"
                angle = (2 * np.pi / self.num_robots) * i
                
                sx = 0.5 * np.cos(angle)
                sy = 0.5 * np.sin(angle)
                syaw = angle
                
                # 目标在圆周上
                goal_radius = random.uniform(8, 12)
                gx = goal_radius * np.cos(angle)
                gy = goal_radius * np.sin(angle)
                
                robot = RobotState(
                    robot_id=robot_id,
                    pose=np.array([sx, sy, syaw]),
                    tether_anchor=np.array([0.0, 0.0, 0.0]),  # 共享中心锚点
                    tether_length=0.0,
                    goal=np.array([gx, gy]),
                    radius=0.2
                )
                robot.max_tether_length = 14.0
                self.robots[robot_id] = robot
        
        else:
            self.get_logger().error(f"未知场景: {scenario_name}")
    
    def cmd_vel_callback(self, msg: Twist, robot_id: str):
        """接收机器人速度命令"""
        if robot_id in self.robots:
            robot = self.robots[robot_id]
            # 限制速度
            vx = np.clip(msg.linear.x, -self.max_linear_velocity, self.max_linear_velocity)
            wz = np.clip(msg.angular.z, -self.max_angular_velocity, self.max_angular_velocity)
            robot.velocity = np.array([vx, wz])
    
    def simulation_step(self):
        """仿真物理步进（50 Hz）"""
        self.step_count += 1
        
        # 对每个机器人进行物理更新
        for robot_id, robot in self.robots.items():
            self._update_robot_dynamics(robot)
            
            # 检测 tether 交叉
            self._check_tether_crossings(robot)
        
        self.sim_time += self.dt
    
    def _update_robot_dynamics(self, robot: RobotState):
        """
        更新单个机器人的动力学
        简单的单轮差分驱动机器人模型
        """
        vx, wz = robot.velocity
        theta = robot.pose[2]
        
        # 计算位置变化
        dx = vx * np.cos(theta) * self.dt
        dy = vx * np.sin(theta) * self.dt
        dtheta = wz * self.dt
        
        # 更新姿态
        new_x = robot.pose[0] + dx
        new_y = robot.pose[1] + dy
        new_theta = robot.pose[2] + dtheta
        
        # 边界检查
        new_x = np.clip(new_x, self.bounds['xmin'], self.bounds['xmax'])
        new_y = np.clip(new_y, self.bounds['ymin'], self.bounds['ymax'])
        
        # 障碍物碰撞检查（简化为点质量）
        if self._check_collision(np.array([new_x, new_y]), robot.radius):
            # 碰撞：停止并反弹
            robot.velocity[0] *= -0.5  # 反向减速
            robot.velocity[1] = 0.0
            # 不更新位置
        else:
            robot.pose[0] = new_x
            robot.pose[1] = new_y
            robot.pose[2] = new_theta
        
        # 更新 tether 物理
        tension, tether_force = TetherPhysics.compute_tension(
            robot.pose, robot.tether_anchor, np.array([vx, 0.0]),
            rest_length=robot.tether_length,
            k=robot.tether_k,
            damping=robot.tether_damping
        )
        
        # 应用 tether 力（影响速度）
        if tension > 0.1:
            # 力转换为加速度
            ax = tether_force[0] / self.robot_mass
            ay = tether_force[1] / self.robot_mass
            
            robot.velocity[0] += ax * self.dt
            robot.velocity[0] = np.clip(robot.velocity[0], -self.max_linear_velocity, self.max_linear_velocity)
        
        robot.tether_tension = tension
        
        # 自动延长 tether（当远离锚点时）
        dist_to_anchor = np.linalg.norm(robot.pose[:2] - robot.tether_anchor[:2])
        robot.tether_length = min(dist_to_anchor, robot.max_tether_length)
        
        # 记录历史
        robot.history.append(robot.pose.copy())
    
    def _check_collision(self, pos: np.ndarray, radius: float) -> bool:
        """检查机器人是否与障碍物碰撞"""
        for obs in self.obstacles:
            if obs['type'] == 'wall':
                # 线段碰撞检测
                wall_x1, wall_y1 = obs['x1'], obs['y1']
                wall_x2, wall_y2 = obs['x2'], obs['y2']
                thickness = obs.get('thickness', 0.2)
                
                # 简化的点到线段距离
                dist = self._point_to_line_distance(pos, wall_x1, wall_y1, wall_x2, wall_y2)
                if dist < (radius + thickness/2):
                    return True
                    
            elif obs['type'] == 'box':
                # AABB 盒碰撞
                half_size = obs['size'] / 2 + radius
                if (abs(pos[0] - obs['x']) < half_size and 
                    abs(pos[1] - obs['y']) < half_size):
                    return True
                    
            elif obs['type'] == 'cylinder':
                # 圆形障碍物
                dist = np.linalg.norm(pos - np.array([obs['x'], obs['y']]))
                if dist < (radius + obs['radius']):
                    return True
        
        return False
    
    def _point_to_line_distance(self, p: np.ndarray, x1: float, y1: float, 
                               x2: float, y2: float) -> float:
        """计算点到线段的距离"""
        # 线段向量
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx*dx + dy*dy
        
        if length_sq < 1e-6:
            return np.linalg.norm(p - np.array([x1, y1]))
        
        # 投影参数 t
        t = ((p[0]-x1)*dx + (p[1]-y1)*dy) / length_sq
        t = np.clip(t, 0, 1)
        
        # 最近点
        nearest = np.array([x1 + t*dx, y1 + t*dy])
        return np.linalg.norm(p - nearest)
    
    def _check_tether_crossings(self, robot: RobotState):
        """检测 tether 是否与其他机器人交叉"""
        if len(robot.history) < 2:
            return
        
        my_segments = self._get_tether_segments(robot)
        if not my_segments:
            return
        
        for other_id, other_robot in self.robots.items():
            if other_id == robot.robot_id:
                continue
            
            other_segments = self._get_tether_segments(other_robot)
            if not other_segments:
                continue
            
            # 检测线段相交
            for seg1 in my_segments:
                for seg2 in other_segments:
                    crossing = self._segments_intersect_2d(seg1, seg2)
                    if crossing:
                        # 记录交叉事件
                        event = {
                            'timestamp': self.sim_time,
                            'robot_a': robot.robot_id,
                            'robot_b': other_id,
                            'crossing_point': crossing,
                            'severity': min(robot.tether_tension, other_robot.tether_tension) / 50.0
                        }
                        self.crossing_events.append(event)
                        self.entanglement_count += 1
                        break
    
    def _get_tether_segments(self, robot: RobotState) -> List[Tuple[np.ndarray, np.ndarray]]:
        """获取 tether 的线段表示（用于交叉检测）"""
        if len(robot.history) < 2:
            return []
        
        segments = []
        anchor = robot.tether_anchor[:2]
        
        # 沿轨迹采样（每隔一定距离取一个点）
        points = [anchor]
        for pose in robot.history:
            points.append(pose[:2])
        
        for i in range(len(points) - 1):
            segments.append((points[i], points[i+1]))
        
        return segments
    
    def _segments_intersect_2d(self, seg1: Tuple[np.ndarray, np.ndarray],
                               seg2: Tuple[np.ndarray, np.ndarray]) -> Optional[np.ndarray]:
        """二维线段相交检测"""
        p1, p2 = seg1
        p3, p4 = seg2
        
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
        
        if ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4):
            # 计算交点
            return (p1 + p2) / 2  # 简化：返回中点
        return None
    
    def publish_sensor_data(self):
        """发布传感器数据（20 Hz）"""
        for robot_id, robot in self.robots.items():
            # 1. 发布 Odometry
            odom = Odometry()
            odom.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
            odom.child_frame_id = 'base_link'
            odom.pose.pose = Pose(
                position=Point(x=robot.pose[0], y=robot.pose[1], z=0.0),
                orientation=Quaternion(
                    z=np.sin(robot.pose[2]/2),
                    w=np.cos(robot.pose[2]/2)
                )
            )
            odom.twist.twist.linear.x = robot.velocity[0]
            odom.twist.twist.angular.z = robot.velocity[1]
            self.odom_publishers[robot_id].publish(odom)
            
            # 2. 发布 LaserScan（模拟）
            scan = self._simulate_lidar(robot)
            self.scan_publishers[robot_id].publish(scan)
            
            # 3. 发布 TetherState（用于 swarm coordination）
            tether_msg = TetherState()
            tether_msg.header = Header(stamp=self.get_clock().now().to_msg())
            tether_msg.robot_id = robot_id
            tether_msg.anchor_pose = PoseStamped()
            tether_msg.anchor_pose.header = tether_msg.header
            tether_msg.anchor_pose.pose.position = Point(
                x=robot.tether_anchor[0],
                y=robot.tether_anchor[1],
                z=0.0
            )
            tether_msg.tension = float(robot.tether_tension)
            tether_msg.slack_length = float(robot.max_tether_length - robot.tether_length)
            tether_msg.current_length = float(robot.tether_length)
            tether_msg.is_crossing = self._is_tether_crossing(robot)
            tether_msg.priority_score = self._compute_priority(robot)
            
            # tether 关键点
            if len(robot.history) > 1:
                # 每 0.5m 采样一个点
                tether_msg.tether_segment_points = []
                step = max(1, len(robot.history) // 10)
                for i in range(0, len(robot.history), step):
                    pt = robot.history[i]
                    tether_msg.tether_segment_points.append(Point(x=pt[0], y=pt[1], z=0.0))
                if robot.history:
                    tether_msg.tether_segment_points.append(
                        Point(x=robot.history[-1][0], y=robot.history[-1][1], z=0.0)
                    )
            
            self.tether_publishers[robot_id].publish(tether_msg)
            
            # 4. 更新 yielding action（从外部接收，这里先简单模拟）
            # TODO: 从 swarm coordinator 接收
    
    def _simulate_lidar(self, robot: RobotState) -> LaserScan:
        """模拟 2D 激光雷达"""
        scan = LaserScan()
        scan.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='laser_link')
        scan.angle_min = self.lidar_angle_min
        scan.angle_max = self.lidar_angle_max
        scan.angle_increment = self.lidar_angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = self.lidar_range
        
        # 生成每个角度的距离读数
        ranges = []
        for i in range(self.lidar_num_readings):
            angle = robot.pose[2] + i * self.lidar_angle_increment
            # 计算射线方向
            dx = np.cos(angle)
            dy = np.sin(angle)
            ray_dir = np.array([dx, dy])
            
            # 射线与障碍物求交
            min_dist = self.lidar_range
            for obs in self.obstacles:
                dist = self._ray_obstacle_intersect(robot.pose[:2], ray_dir, obs)
                if dist is not None and dist < min_dist:
                    min_dist = dist
            
            ranges.append(float(min_dist))
        
        scan.ranges = ranges
        return scan
    
    def _ray_obstacle_intersect(self, origin: np.ndarray, direction: np.ndarray, 
                               obs: Dict) -> Optional[float]:
        """射线与障碍物相交检测（简化）"""
        # 这里实现简化的相交检测
        # 完整实现需要针对每种障碍物类型
        return None  # 简化：返回 None 表示无碰撞
    
    def _is_tether_crossing(self, robot: RobotState) -> bool:
        """检查机器人 tether 是否正在交叉"""
        for crossing in self.crossing_events[-10:]:  # 检查最近的交叉
            if crossing['robot_a'] == robot.robot_id or crossing['robot_b'] == robot.robot_id:
                return True
        return False
    
    def _compute_priority(self, robot: RobotState) -> float:
        """计算机器人优先级"""
        # 距离目标的距离
        dist_to_goal = np.linalg.norm(robot.pose[:2] - robot.goal)
        dist_factor = max(0, 1.0 - dist_to_goal / 20.0)
        
        # 张力因子
        tension_factor = min(1.0, robot.tether_tension / 50.0)
        
        return 0.5 + 0.3 * dist_factor + 0.2 * tension_factor
    
    def publish_metrics(self):
        """定期发布 swarm metrics"""
        metrics = SwarmMetrics()
        metrics.header = Header(stamp=self.get_clock().now().to_msg())
        metrics.active_robots = len(self.robots)
        metrics.total_entanglements = self.entanglement_count
        metrics.resolved_entanglements = len(self.crossing_events)
        
        # 计算平均和最大张力
        tensions = [r.tether_tension for r in self.robots.values()]
        metrics.avg_tension = float(np.mean(tensions)) if tensions else 0.0
        metrics.max_tension = float(np.max(tensions)) if tensions else 0.0
        
        # 检查是否所有机器人都接近目标
        all_at_goal = True
        for robot in self.robots.values():
            dist = np.linalg.norm(robot.pose[:2] - robot.goal)
            if dist > 1.0:  # 1m 阈值
                all_at_goal = False
                break
        metrics.all_robots_at_goal = all_at_goal
        
        self.metrics_pub.publish(metrics)
    
    def shutdown(self):
        """关闭仿真器"""
        # 保存最终数据
        self._save_final_data()
        
        # 清理
        self.destroy_node()
    
    def _save_final_data(self):
        """保存仿真数据"""
        # 创建结果目录
        results_dir = Path(f"/home/jetson/swarm_simulation_results/{self.scenario_name}")
        results_dir.mkdir(parents=True, exist_ok=True)
        
        # 保存交叉事件
        crossing_file = results_dir / 'crossing_events.json'
        with open(crossing_file, 'w') as f:
            json.dump(self.crossing_events, f, indent=2, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x)
        
        # 保存每个机器人的轨迹
        for robot_id, robot in self.robots.items():
            traj_file = results_dir / f'trajectory_{robot_id}.csv'
            with open(traj_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['step', 'x', 'y', 'theta', 'tension'])
                for step, pose in enumerate(robot.history):
                    writer.writerow([step, pose[0], pose[1], pose[2], robot.tether_tension])
        
        # 摘要
        summary = {
            'scenario': self.scenario_name,
            'num_robots': self.num_robots,
            'total_crossings': len(self.crossing_events),
            'entanglements': self.entanglement_count,
            'sim_duration': self.sim_time
        }
        
        summary_file = results_dir / 'summary.json'
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        self.get_logger().info(f"仿真数据已保存到: {results_dir}")


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    if len(sys.argv) < 3:
        print("用法: swarm_simulator.py <scenario> <num_robots>")
        print("场景: bottleneck, crossing, expansion")
        sys.exit(1)
    
    scenario = sys.argv[1]
    num_robots = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    
    simulator = SwarmSimulator(scenario, num_robots)
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("仿真结束")
    finally:
        simulator.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
