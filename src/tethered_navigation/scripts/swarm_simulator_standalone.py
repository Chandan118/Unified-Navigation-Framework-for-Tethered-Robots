#!/usr/bin/env python3
"""
Swarm Robotics Simulator for Tethered Robots
=============================================
独立运行的群体机器人仿真器，专为npj Robotics论文设计

无需编译ROS 2包，纯Python实现，支持：
- Baseline模式：独立导航，无视其他机器人的tether
- Cooperative模式：通过/swarm_tether_states通信，协同避免纠缠

模拟物理：
- 2D差分驱动机器人运动学
- Tether弹簧-质点模型（张力、松弛）
- Tether-tether交叉检测
- Fuzzy逻辑控制器（简化版）
- DRL奖励函数（含inter-robot entanglement penalty）

输出CSV数据（论文所需）：
- swarm_metrics.csv      : 群体级指标
- trajectory_robot_X.csv : 每个机器人的轨迹和tension历史
- crossing_events.csv    : tether交叉事件
- yielding_decisions.csv : yield/reroute决策记录
"""

import numpy as np
import math
import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field
from enum import Enum
import random

# ==================== Configuration ====================

SIMULATION_DT = 0.1  # 仿真步长 (s) - 10 Hz
MAX_SIMULATION_TIME = 300.0  # 最大仿真时间 (5分钟)
TETHER_K = 100.0    # 弹簧刚度 (N/m)
TETHER_DAMPING = 5.0  # 阻尼系数
ROBOT_RADIUS = 0.3   # 机器人半径 (m)
MAX_TETHER_LENGTH = 10.0  # 最大tether长度 (m)
COLLISION_DISTANCE = 0.6  # 机器人间最小距离 (m)

# Fuzzy controller parameters
MAX_SPEED = 1.0  # m/s (increased for faster traversal)
MIN_SPEED = 0.1
SAFE_DISTANCE = 0.8  # 激光雷达安全距离 (m)
CRITICAL_DISTANCE = 0.4  # 危险距离 (m)

# Yielding parameters
YIELD_SPEED_REDUCTION = 0.3  # 让行时减速到30%
REROUTE_ANGLE_MAX = math.pi/4  # 最大偏转角度
COOLDOWN_AFTER_YIELD = 2.0  # 让行后冷却时间 (s) - reduced

# ==================== Data Structures ====================

class YieldingAction(Enum):
    MAINTAIN = "maintain"
    YIELD = "yield"
    REROUTE = "reroute"
    SPEED_UP = "speed_up"

@dataclass
class RobotState:
    """单个机器人的完整状态"""
    robot_id: str
    pose: np.ndarray  # [x, y, theta]
    velocity: np.ndarray  # [vx, vy]
    tether_anchor: np.ndarray  # 锚点 [x, y]
    tether_tension: float = 0.0
    tether_length: float = 0.0
    max_tether_length: float = 10.0  # 最大tether长度 (m)
    goal: np.ndarray = field(default_factory=lambda: np.zeros(2))
    tether_history: List[np.ndarray] = field(default_factory=list)  # tether关键点
    current_yielding: YieldingAction = YieldingAction.MAINTAIN
    last_yield_time: float = -10.0
    priority_score: float = 0.5
    reached_goal: bool = False
    collision_count: int = 0
    total_distance_traveled: float = 0.0

    def distance_to_goal(self) -> float:
        return np.linalg.norm(self.pose[:2] - self.goal)

    def distance_to_anchor(self) -> float:
        return np.linalg.norm(self.pose[:2] - self.tether_anchor[:2])

@dataclass
class SwarmMetrics:
    """群体级指标（每秒记录）"""
    timestamp: float
    active_robots: int
    total_entanglements: int
    resolved_entanglements: int
    avg_tension: float
    max_tension: float
    swarm_traversal_time: float  # 从开始到全部到达的时间
    all_at_goal: bool
    entanglement_pairs: List[str]

@dataclass
class CrossingEvent:
    """tether交叉事件"""
    timestamp: float
    robot_a: str
    robot_b: str
    crossing_x: float
    crossing_y: float
    severity: float  # 0-1，交叉���重程度

@dataclass
class YieldingDecision:
    """yield决策记录"""
    timestamp: float
    requester: str  # 请求让行的机器人
    target: str     # 被请求让行的机器人
    crossing_x: float
    crossing_y: float
    severity: float
    suggested_action: str
    yield_duration_est: float
    priority_comparison: float
    actual_action: str
    accepted: bool

# ==================== Physics Models ====================

class TetherPhysics:
    """Tether物理模型（弹簧-质点）"""
    
    @staticmethod
    def compute_tension(robot_pos: np.ndarray, anchor: np.ndarray,
                       velocity: np.ndarray, current_tether_length: float,
                       max_tether_length: float, k: float, damping: float) -> Tuple[float, np.ndarray]:
        """
        计算tether张力和作用在机器人上的力
        
        模型：tether有一个松弛长度（slack = max_tether_length * 0.7）
        只有当当前长度超过松弛长度时，才开始产生张力
        
        Returns:
            tension: 张力大小 (N)
            force: 力向量 [fx, fy]
        """
        displacement = robot_pos[:2] - anchor[:2]
        current_length = np.linalg.norm(displacement)
        
        if current_length < 1e-6:
            return 0.0, np.zeros(2)
        
        direction = displacement / current_length
        
        # 松弛长度：最大长度的70%范围内无张力
        slack_length = max_tether_length * 0.7
        
        if current_length <= slack_length:
            # 未拉紧，无张力
            return 0.0, np.zeros(2)
        
        # 超过松弛长度后，弹簧力开始作用
        extension = current_length - slack_length
        spring_force = k * extension
        
        # 阻尼力（仅当拉伸时）
        radial_velocity = np.dot(velocity[:2], direction)
        damping_force = damping * max(0, radial_velocity)
        
        tension = spring_force + damping_force
        force = -direction * tension  # 指向锚点
        
        return tension, force

    @staticmethod
    def compute_tether_segments(robot_pos: np.ndarray, anchor: np.ndarray, 
                               segment_spacing: float = 0.2) -> List[Tuple[float, float]]:
        """
        生成tether的关键采样点（用于交叉检测）
        
        Returns:
            segments: 线段端点列表 [(x1,y1), (x2,y2), ...]
        """
        displacement = anchor[:2] - robot_pos[:2]
        length = np.linalg.norm(displacement)
        direction = displacement / (length + 1e-6)
        
        segments = []
        current = robot_pos[:2].copy()
        
        # 从机器人到锚点采样
        steps = int(length / segment_spacing)
        for i in range(steps):
            next_pt = current + direction * segment_spacing
            segments.append((current[0], current[1], next_pt[0], next_pt[1]))
            current = next_pt
        
        # 最后一段到锚点
        segments.append((current[0], current[1], anchor[0], anchor[1]))
        
        return segments

# ==================== Fuzzy Controller (Simplified) ====================

class FuzzyController:
    """
    简化的模糊逻辑控制器
    输入：前方障碍物距离、tension、相对目标方向
    输出：速度修正因子、转向角修正
    """
    
    def __init__(self):
        self.safe_dist = SAFE_DISTANCE
        self.critical_dist = 0.3
        
    def compute(self, robot: RobotState, obstacle_distance: float, 
                other_tether_segments: List[Tuple]) -> Tuple[float, float]:
        """
        计算控制输出
        
        Returns:
            speed_factor: 速度因子 (0-1)
            heading_offset: 转向偏移 (rad)
        """
        # 1. 障碍物规避（激光雷达）
        if obstacle_distance < CRITICAL_DISTANCE:
            speed_factor = 0.2  # 危险但非完全停止
            heading_offset = np.random.uniform(-math.pi/2, math.pi/2)
        elif obstacle_distance < SAFE_DISTANCE:
            # 线性减速区间
            speed_factor = 0.3 + 0.7 * (obstacle_distance - CRITICAL_DISTANCE) / (SAFE_DISTANCE - CRITICAL_DISTANCE)
            heading_offset = 0.5 if random.random() > 0.5 else -0.5
        else:
            speed_factor = 1.0
            heading_offset = 0.0
        
        # 2. Tension惩罚
        if robot.tether_tension > 20.0:  # 高张力
            speed_factor *= 0.6
        elif robot.tether_tension > 10.0:
            speed_factor *= 0.85
        
        # 3. Yielding行为
        if robot.current_yielding == YieldingAction.YIELD:
            speed_factor *= YIELD_SPEED_REDUCTION
        elif robot.current_yielding == YieldingAction.REROUTE:
            speed_factor *= 0.7
            heading_offset = np.random.uniform(-REROUTE_ANGLE_MAX, REROUTE_ANGLE_MAX)
        
        return np.clip(speed_factor, 0.0, 1.0), heading_offset

# ==================== Swarm Coordinator ====================

class SwarmCoordinator:
    """
    分布式群体协调器（每个机器人运行一个实例）
    功能：
    1. 监听所有机器人的tether状态
    2. 检测本机tether与其他tether的交叉
    3. 基于优先级决定谁应该让行
    4. 发布yielding决策
    """
    
    def __init__(self, robot_id: str, num_robots: int, mode: str = 'cooperative'):
        self.robot_id = robot_id
        self.mode = mode  # 'baseline' or 'cooperative'
        self.num_robots = num_robots
        
        # 存储所有机器人的tether状态
        self.tether_states: Dict[str, RobotState] = {}
        
        # 优先级权重
        self.priority_weights = {
            'distance_to_goal': 0.5,
            'tension': 0.3,
            'static': 0.2
        }
        
        # 历史决策（避免频繁切换）
        self.decision_history: List[YieldingDecision] = []
        self.last_decision_time = -10.0
        
        # 指标统计
        self.crossings_detected = 0
        self.yields_executed = 0
        self.entanglements_avoided = 0
        
    def update_tether_state(self, robot_id: str, state: RobotState):
        """更新机器人tether状态"""
        self.tether_states[robot_id] = state
    
    def compute_priority(self, robot_id: str) -> float:
        """计算机器人优先级（0-1，越高越优先）"""
        if robot_id not in self.tether_states:
            return 0.5
        
        state = self.tether_states[robot_id]
        
        # 1. 距离目标距离（越近越高）
        dist_to_goal = state.distance_to_goal()
        max_dist = 20.0  # 假设最大距离
        priority_goal = max(0.0, 1.0 - (dist_to_goal / max_dist))
        
        # 2. Tension（张力越大越紧急）
        tension_factor = min(1.0, state.tether_tension / 30.0)
        
        # 3. 静态优先级（基于ID）
        static_priority = (int(robot_id.split('_')[1]) / self.num_robots)
        
        # 加权组合
        priority = (self.priority_weights['distance_to_goal'] * priority_goal +
                   self.priority_weights['tension'] * tension_factor +
                   self.priority_weights['static'] * static_priority)
        
        return np.clip(priority, 0.0, 1.0)
    
    def detect_crossings(self, my_state: RobotState) -> List[Tuple[str, float, Tuple]]:
        """
        检测本机tether与其他机器人tether的交叉
        
        Returns:
            List of (other_robot_id, severity, crossing_point)
        """
        if self.mode == 'baseline':
            return []  # baseline模式不检测交叉
        
        my_segments = TetherPhysics.compute_tether_segments(
            my_state.pose, my_state.tether_anchor
        )
        
        crossings = []
        
        for other_id, other_state in self.tether_states.items():
            if other_id == self.robot_id:
                continue
            
            other_segments = TetherPhysics.compute_tether_segments(
                other_state.pose, other_state.tether_anchor
            )
            
            # 检测线段相交
            for seg_a in my_segments:
                for seg_b in other_segments:
                    severity, point = self._segment_crossing_severity(seg_a, seg_b)
                    if severity > 0:
                        crossings.append((other_id, severity, point))
                        self.crossings_detected += 1
        
        return crossings
    
    def _segment_crossing_severity(self, seg_a: Tuple, seg_b: Tuple) -> Tuple[float, Tuple]:
        """
        计算两条线段的交叉严重程度
        
        Returns:
            severity: 0-1，0表示无交叉，1表示严重交叉
            point: 交叉点 (x, y) 或 None
        """
        # 简化：使用线段相交检测
        x1, y1, x2, y2 = seg_a
        x3, y3, x4, y4 = seg_b
        
        # 向量化
        da = np.array([x2 - x1, y2 - y1])
        db = np.array([x4 - x3, y4 - y3])
        
        # 行列式
        det = da[0] * db[1] - da[1] * db[0]
        if abs(det) < 1e-6:
            return 0.0, None  # 平行
        
        # 计算交点参数
        s = ((x3 - x1) * db[1] - (y3 - y1) * db[0]) / det
        t = ((x3 - x1) * da[1] - (y3 - y1) * da[0]) / det
        
        if 0 <= s <= 1 and 0 <= t <= 1:
            # 相交
            intersection = np.array([x1 + s * da[0], y1 + s * da[1]])
            
            # 计算严重程度：基于交叉点距离两机器人锚点的相对位置
            # 交叉越靠近机器人（tether末端），越严重
            severity = 0.5 * (s + t)  # 简单启发式
            return severity, (intersection[0], intersection[1])
        
        return 0.0, None
    
    def make_yielding_decision(self, my_state: RobotState, 
                               crossings: List[Tuple]) -> YieldingAction:
        """
        基于交叉检测和优先级做出yield决策
        
        Returns:
            action: YieldingAction枚举
        """
        if self.mode == 'baseline' or not crossings:
            return YieldingAction.MAINTAIN
        
        current_time = time.time()
        
        # 检查冷却时间
        if current_time - my_state.last_yield_time < COOLDOWN_AFTER_YIELD:
            return YieldingAction.MAINTAIN
        
        # 选择最严重的交叉
        worst_crossing = max(crossings, key=lambda x: x[1])  # severity最大
        other_id, severity, crossing_point = worst_crossing
        
        # 计算优先级比较
        my_priority = self.compute_priority(self.robot_id)
        other_priority = self.compute_priority(other_id)
        priority_diff = my_priority - other_priority
        
        # 改进的决策规则（避免死锁）：
        # 1. 如果对方优先级明显更高（>0.15），我方yield
        # 2. 如果我方优先级明显更高（>0.15），我方maintain（让对方yield）
        # 3. 如果优先级接近（平局），基于ID决定：ID小的yield（打破对称）
        
        YIELD_THRESHOLD = 0.15
        
        if my_priority < other_priority - YIELD_THRESHOLD:
            # 对方优先级明显更高，我方让行
            action = YieldingAction.YIELD
            self.yields_executed += 1
            my_state.last_yield_time = current_time
        elif my_priority > other_priority + YIELD_THRESHOLD:
            # 我方优先级明显更高，保持原样（期望对方让行）
            action = YieldingAction.MAINTAIN
        else:
            # 优先级接近 - 使用确定性规则打破平局
            # 规则：ID较小的机器人yield（避免对称僵局）
            my_id_num = int(self.robot_id.split('_')[1])
            other_id_num = int(other_id.split('_')[1])
            
            if my_id_num < other_id_num:
                # 我方ID较小，让行
                action = YieldingAction.YIELD
                self.yields_executed += 1
                my_state.last_yield_time = current_time
            else:
                # 对方ID较小，期望对方让行
                action = YieldingAction.MAINTAIN
        
        return action

# ==================== Robot Controller (包含DRL奖励) ====================

class RobotController:
    """
    单个机器人的控制器
    整合：
    - 运动学模型
    - Fuzzy逻辑
    - Swarm协调
    - DRL奖励计算（含inter-robot entanglement penalty）
    """
    
    def __init__(self, robot: RobotState, coordinator: SwarmCoordinator, 
                 fuzzy_ctrl: FuzzyController, mode: str = 'cooperative'):
        self.robot = robot
        self.coordinator = coordinator
        self.fuzzy_ctrl = fuzzy_ctrl
        self.mode = mode
        
        # DRL奖励权重
        self.reward_weights = {
            'progress': 1.0,           # 向目标前进奖励
            'tension_penalty': -0.5,   # tension惩罚
            'entanglement_penalty': -5.0,  # 纠缠惩罚（关键！）
            'yield_penalty': -0.2,     # 让行惩罚（时间损失）
            'goal_reward': 100.0        # 到达目标奖励
        }
        
        self.total_reward = 0.0
        self.episode_steps = 0
        
    def step(self, obstacle_distances: Dict[str, float], all_robot_states: Dict[str, RobotState]):
        """
        执行一个仿真步
        
        Returns:
            cmd_vel: [vx, vy]
            reward: 本步奖励
        """
        self.episode_steps += 1
        
        # 1. 更新协调器中的状态（用于通信）
        for rid, state in all_robot_states.items():
            self.coordinator.update_tether_state(rid, state)
        
        # 2. 检测交叉
        crossings = self.coordinator.detect_crossings(self.robot)
        
        # 3. 做出yield决策
        if self.mode == 'cooperative':
            action = self.coordinator.make_yielding_decision(self.robot, crossings)
            self.robot.current_yielding = action
        else:
            self.robot.current_yielding = YieldingAction.MAINTAIN
        
        # 4. 计算基础控制（向��标）
        base_cmd = self._compute_navigation_command()
        
        # 5. Fuzzy修正
        # 获取前方障碍物距离（简化：取多个方向的最小值）
        front_dist = obstacle_distances.get('front', SAFE_DISTANCE + 1.0)
        speed_factor, heading_offset = self.fuzzy_ctrl.compute(
            self.robot, front_dist, []
        )
        
        # 6. 应用yield修正
        if self.robot.current_yielding == YieldingAction.YIELD:
            speed_factor *= 0.3
        elif self.robot.current_yielding == YieldingAction.REROUTE:
            speed_factor *= 0.6
        
        # 7. 最终命令
        v = base_cmd[0] * speed_factor
        theta = base_cmd[1] + heading_offset
        
        cmd_vel = np.array([v * math.cos(theta), v * math.sin(theta)])
        
        # 8. 计算奖励（用于DRL评估）
        reward = self._compute_reward(crossings)
        self.total_reward += reward
        
        return cmd_vel, reward
    
    def _compute_navigation_command(self) -> np.ndarray:
        """计算向目标的导航命令"""
        to_goal = self.robot.goal - self.robot.pose[:2]
        distance = np.linalg.norm(to_goal)
        
        if distance < 0.3:  # 到达目标
            self.robot.reached_goal = True
            return np.array([0.0, 0.0])
        
        # 简单的比例控制
        speed = min(MAX_SPEED, 0.5 * distance)
        heading = math.atan2(to_goal[1], to_goal[0])
        
        # 转换为世界坐标系下的速度
        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)
        
        return np.array([vx, vy])
    
    def _compute_reward(self, crossings: List[Tuple]) -> float:
        """计算DRL奖励函数（含inter-robot entanglement penalty）"""
        reward = 0.0
        
        # 前进奖励
        progress = np.linalg.norm(self.robot.velocity) * SIMULATION_DT
        reward += self.reward_weights['progress'] * progress
        
        # Tension惩罚
        tension_penalty = max(0, self.robot.tether_tension - 10.0) / 10.0
        reward += self.reward_weights['tension_penalty'] * tension_penalty
        
        # 纠缠惩罚（关键！）
        if self.robot.tether_tension > 25.0:  # 高张力表示可能纠缠
            reward += self.reward_weights['entanglement_penalty']
        
        # 让行惩罚
        if self.robot.current_yielding == YieldingAction.YIELD:
            reward += self.reward_weights['yield_penalty']
        
        # 到达目标奖励
        if self.robot.reached_goal:
            reward += self.reward_weights['goal_reward']
        
        return reward

# ==================== Main Simulation Engine ====================

class SwarmSimulation:
    """
    主仿真引擎
    管理所有机器人、物理步进、数据记录
    """
    
    def __init__(self, scenario_name: str, num_robots: int, mode: str = 'cooperative'):
        self.scenario_name = scenario_name
        self.num_robots = num_robots
        self.mode = mode
        
        # 创建场景
        self.scenario = self._create_scenario(scenario_name, num_robots)
        
        # 初始化机器人
        self.robots: Dict[str, RobotState] = {}
        self.controllers: Dict[str, RobotController] = {}
        
        # 根据场景设置合适的tether最大长度
        scenario_max_tether = {
            'bottleneck': 12.0,   # 走廊宽4m，长度18m，12m足够
            'crossing': 25.0,     # 对角线距离~22.6m，需要更大
            'expansion': 15.0,    # 从中心到边缘~12m，15m足够
        }
        max_tether = scenario_max_tether.get(scenario_name, 10.0)
        
        for i in range(num_robots):
            robot_id = f"robot_{i+1}"
            start_pose, goal = self.scenario.robot_start_poses[i], self.scenario.robot_goals[i]
            
            state = RobotState(
                robot_id=robot_id,
                pose=np.array([start_pose[0], start_pose[1], start_pose[2]]),
                velocity=np.zeros(2),
                tether_anchor=np.array([start_pose[0], start_pose[1], 0.0]),  # 锚点在起点
                goal=np.array([goal[0], goal[1]]),
                priority_score=(i+1)/num_robots  # 基于ID的静态优先级
            )
            # 设置最大tether长度（关键！）
            state.max_tether_length = max_tether
            self.robots[robot_id] = state
        
        # 初始化协调器和控制器
        self.coordinator = SwarmCoordinator("master", num_robots, mode)
        self.fuzzy_ctrl = FuzzyController()
        
        for robot_id, robot in self.robots.items():
            coordinator = SwarmCoordinator(robot_id, num_robots, mode)
            controller = RobotController(robot, coordinator, self.fuzzy_ctrl, mode)
            self.controllers[robot_id] = controller
        
        # 数据记录器
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = Path(f"/home/jetson/swarm_simulation_results/{timestamp}_{scenario_name}_{mode}_{num_robots}robots")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.data_logger = self._setup_data_logger()
        
        # 仿真状态
        self.current_time = 0.0
        self.total_entanglements = 0
        self.entanglement_events: List[CrossingEvent] = []
        self.yielding_decisions: List[YieldingDecision] = []
        self.start_time = None
        
        # 障碍物配置（根据场景）
        self.obstacles = self._generate_obstacles()
        
        print(f"✅ 仿真初始化完成: {scenario_name}, {num_robots} robots, mode={mode}")
    
    def _create_scenario(self, name: str, num_robots: int):
        """创建场景配置"""
        if name == 'bottleneck':
            return BottleneckScenario(num_robots)
        elif name == 'crossing':
            return CrossingPathsScenario(num_robots)
        elif name == 'expansion':
            return ExpansionScenario(num_robots)
        else:
            raise ValueError(f"Unknown scenario: {name}")
    
    def _generate_obstacles(self) -> List[Tuple[float, float, float]]:
        """根据场景生成障碍物"""
        if self.scenario_name == 'bottleneck':
            # 狭窄走廊：两面墙
            obstacles = []
            # 上墙 y=2.5, x从-8到8
            for x in np.arange(-8.0, 8.1, 0.5):
                obstacles.append((x, 2.5, 0.5))  # x, y, radius
                obstacles.append((x, -2.5, 0.5))
            return obstacles
        elif self.scenario_name == 'crossing':
            # Crossing paths: 开放空间，无中心障碍
            # 允许机器人自由交叉，突出tether协调
            obstacles = []  # 无静态障碍
            return obstacles
        elif self.scenario_name == 'expansion':
            # 随机散布的柱子
            obstacles = []
            random.seed(42)
            for _ in range(30):
                x = random.uniform(-10, 10)
                y = random.uniform(-10, 10)
                # 避开中心区域（锚点）
                if np.linalg.norm([x, y]) > 3.0:
                    obstacles.append((x, y, random.uniform(0.3, 0.8)))
            return obstacles
        return []
    
    def _setup_data_logger(self):
        """设置CSV数据记录器"""
        # 1. Swarm metrics
        self.metrics_file = open(self.log_dir / 'swarm_metrics.csv', 'w', newline='')
        self.metrics_writer = csv.writer(self.metrics_file)
        self.metrics_writer.writerow([
            'timestamp', 'active_robots', 'total_entanglements',
            'resolved_entanglements', 'avg_tension', 'max_tension',
            'swarm_traversal_time', 'all_at_goal', 'entanglement_pairs'
        ])
        
        # 2. Robot trajectories (每个机器人一个文件)
        self.traj_files = {}
        self.traj_writers = {}
        for robot_id in self.robots:
            f = open(self.log_dir / f'trajectory_{robot_id}.csv', 'w', newline='')
            w = csv.writer(f)
            w.writerow(['timestamp', 'x', 'y', 'theta', 'linear_x', 'angular_z',
                       'tension', 'tether_length', 'yielding_action', 'distance_to_goal', 'reward'])
            self.traj_files[robot_id] = f
            self.traj_writers[robot_id] = w
        
        # 3. Crossing events
        self.crossing_file = open(self.log_dir / 'crossing_events.csv', 'w', newline='')
        self.crossing_writer = csv.writer(self.crossing_file)
        self.crossing_writer.writerow([
            'timestamp', 'robot_a', 'robot_b', 'crossing_x', 'crossing_y', 'severity'
        ])
        
        # 4. Yielding decisions
        self.yielding_file = open(self.log_dir / 'yielding_decisions.csv', 'w', newline='')
        self.yielding_writer = csv.writer(self.yielding_file)
        self.yielding_writer.writerow([
            'timestamp', 'requester', 'target', 'crossing_x', 'crossing_y',
            'severity', 'suggested_action', 'yield_duration_est',
            'priority_comparison', 'actual_action', 'accepted'
        ])
        
        # 5. Metadata
        self.metadata = {
            'scenario': self.scenario_name,
            'mode': self.mode,
            'num_robots': self.num_robots,
            'start_time': None,
            'end_time': None,
            'max_time': MAX_SIMULATION_TIME,
            'simulation_dt': SIMULATION_DT
        }
    
    def run(self):
        """运行主仿真循环"""
        print(f"\n🚀 开始仿真: {self.scenario_name} - {self.mode} mode")
        print(f"   机器人数量: {self.num_robots}")
        print(f"   最大时间: {MAX_SIMULATION_TIME}s")
        
        self.start_time = time.time()
        self.metadata['start_time'] = datetime.now().isoformat()
        
        step = 0
        last_log_time = 0.0
        
        try:
            while self.current_time < MAX_SIMULATION_TIME:
                step_start = time.time()
                
                # 检查是否全部到达目标
                all_at_goal = all(r.reached_goal for r in self.robots.values())
                if all_at_goal:
                    print(f"✅ 所有机器人已到达目标，仿真结束 (t={self.current_time:.1f}s)")
                    break
                
                # 物理步进
                self._physics_step()
                
                # 数据记录（每0.5秒记录一次metrics）
                if self.current_time - last_log_time >= 0.5:
                    self._log_metrics()
                    last_log_time = self.current_time
                
                # 详细轨迹记录（每步）
                self._log_trajectories(step)
                
                # 时间推进
                self.current_time += SIMULATION_DT
                step += 1
                
                # 实时显示
                if step % 100 == 0:
                    active = sum(1 for r in self.robots.values() if not r.reached_goal)
                    avg_tension = np.mean([r.tether_tension for r in self.robots.values()])
                    print(f"  t={self.current_time:.1f}s  active={active}/{self.num_robots}  "
                          f"avg_tension={avg_tension:.1f}N  entanglements={self.total_entanglements}")
                
                # 保持实时速度
                elapsed = time.time() - step_start
                if elapsed < SIMULATION_DT:
                    time.sleep(SIMULATION_DT - elapsed)
                    
        except KeyboardInterrupt:
            print("\n⚠️  仿真被用户中断")
        
        finally:
            self._finalize()
    
    def _physics_step(self):
        """物理仿真一步"""
        # 1. 计算所有机器人的tether力和状态
        tether_forces = {}
        for robot_id, robot in self.robots.items():
            if not robot.reached_goal:
                tension, force = TetherPhysics.compute_tension(
                    robot.pose, robot.tether_anchor,
                    robot.velocity, robot.tether_length,
                    robot.max_tether_length,  # 新增：传递最大tether长度
                    TETHER_K, TETHER_DAMPING
                )
                robot.tether_tension = tension
                robot.tether_length = robot.distance_to_anchor()
                tether_forces[robot_id] = force
        
        # 2. 获取每个机器人的前方障碍物距离（前方扇形检测）
        obstacle_info = {}
        for robot_id, robot in self.robots.items():
            if robot.reached_goal:
                continue
            
            # 前方扇形检测（±60度）
            front_angle_half = math.pi / 3  # 60度
            min_front = float('inf')
            
            robot_x, robot_y, robot_theta = robot.pose
            
            for ox, oy, orad in self.obstacles:
                # 计算障碍物相对于机器人的位置
                dx = ox - robot_x
                dy = oy - robot_y
                dist_to_obstacle = math.sqrt(dx*dx + dy*dy) - orad - ROBOT_RADIUS
                
                if dist_to_obstacle < 0:
                    continue  # 已经碰撞
                
                # 计算障碍物相对于机器人朝向的角度
                angle_to_obstacle = math.atan2(dy, dx)
                angle_diff = abs((angle_to_obstacle - robot_theta + math.pi) % (2*math.pi) - math.pi)
                
                # 只考虑前方扇形内的障碍物
                if angle_diff <= front_angle_half:
                    min_front = min(min_front, dist_to_obstacle)
            
            # 如果前方没有障碍物，给一个很大的值
            if min_front == float('inf'):
                min_front = SAFE_DISTANCE + 5.0
            
            obstacle_info[robot_id] = {'front': min_front}
        
        # 3. 为每个机器人计算控制命令
        commands = {}
        rewards = {}
        for robot_id, robot in self.robots.items():
            if robot.reached_goal:
                commands[robot_id] = np.zeros(2)
                rewards[robot_id] = 0.0
                continue
            
            controller = self.controllers[robot_id]
            cmd, reward = controller.step(obstacle_info[robot_id], self.robots)
            commands[robot_id] = cmd
            rewards[robot_id] = reward
        
        # 4. 应用运动和碰撞处理
        for robot_id, robot in self.robots.items():
            if robot.reached_goal:
                continue
            
            cmd = commands[robot_id]
            
            # 应用tether力（作为额外的速度修正）
            # tether力指向锚点，会拉动机器人
            tether_force = tether_forces.get(robot_id, np.zeros(2))
            
            # 将力转换为加速度（简单模型：F=ma，假设质量=1kg）
            acceleration = tether_force  # m/s^2
            
            # 融合导航命令和tether力
            # 导航命令是期望速度，tether力是加速度
            # 新速度 = 导航速度 + 加速度 * dt
            velocity_cmd = cmd + acceleration * SIMULATION_DT
            
            # 限制速度
            speed = np.linalg.norm(velocity_cmd)
            if speed > MAX_SPEED:
                velocity_cmd = velocity_cmd / speed * MAX_SPEED
            
            # 更新位置（欧拉积分）
            robot.pose[:2] += velocity_cmd * SIMULATION_DT
            robot.velocity = velocity_cmd.copy()
            
            # 更新朝向
            if np.linalg.norm(velocity_cmd) > 0.01:
                robot.pose[2] = math.atan2(velocity_cmd[1], velocity_cmd[0])
            
            # 记录行程距离
            robot.total_distance_traveled += np.linalg.norm(velocity_cmd) * SIMULATION_DT
            
            # 机器人间碰撞检测
            self._check_robot_collisions(robot_id)
            
            # 检查到达目标
            if robot.distance_to_goal() < 0.5:
                robot.reached_goal = True
                print(f"  🎯 {robot_id} 到达目标!")
            
            # 检查纠缠（tether张力过大）
            if robot.tether_tension > 30.0:  # 纠缠阈值
                self.total_entanglements += 1
                # 记录交叉事件（简化：找到造成高张力的原因）
                self._log_entanglement(robot_id)
        
        # 5. 记录yielding决策
        for robot_id, controller in self.controllers.items():
            for decision in controller.coordinator.decision_history[-10:]:  # 只记录最近的
                if decision not in self.yielding_decisions:
                    self.yielding_decisions.append(decision)
                    self.yielding_writer.writerow([
                        round(decision.timestamp, 10), decision.requester, decision.target,
                        float(decision.crossing_x), float(decision.crossing_y),
                        float(decision.severity), decision.suggested_action,
                        float(decision.yield_duration_est), float(decision.priority_comparison),
                        decision.actual_action, bool(decision.accepted)
                    ])
                    self.yielding_file.flush()  # 强制刷新
    
    def _check_robot_collisions(self, robot_id: str):
        """检测机器人间碰撞"""
        robot_a = self.robots[robot_id]
        for other_id, robot_b in self.robots.items():
            if other_id == robot_id:
                continue
            dist = np.linalg.norm(robot_a.pose[:2] - robot_b.pose[:2])
            if dist < COLLISION_DISTANCE:
                robot_a.collision_count += 1
                # 简单的弹性碰撞响应
                direction = (robot_a.pose[:2] - robot_b.pose[:2]) / (dist + 1e-6)
                robot_a.pose[:2] += direction * 0.2
    
    def _log_entanglement(self, robot_id: str):
        """记录纠缠事件"""
        robot = self.robots[robot_id]
        # 找到张力最大的交叉对（简化）
        # 这里只是示例，实际需要精确检测
        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue
            # 检查tether是否交叉
            my_segs = TetherPhysics.compute_tether_segments(robot.pose, robot.tether_anchor)
            other_segs = TetherPhysics.compute_tether_segments(other.pose, other.tether_anchor)
            
            for seg_a in my_segs:
                for seg_b in other_segs:
                    severity, point = self._check_segment_crossing(seg_a, seg_b)
                    if severity > 0.5:
                        event = CrossingEvent(
                            timestamp=self.current_time,
                            robot_a=robot_id,
                            robot_b=other_id,
                            crossing_x=point[0],
                            crossing_y=point[1],
                            severity=severity
                        )
                        self.entanglement_events.append(event)
                        self.crossing_writer.writerow([
                            round(self.current_time, 10), robot_id, other_id,
                            float(point[0]), float(point[1]), float(severity)
                        ])
                        self.crossing_file.flush()  # 强制刷新
                        break
    
    def _check_segment_crossing(self, seg_a, seg_b) -> Tuple[float, np.ndarray]:
        """简化的线段相交检测"""
        x1, y1, x2, y2 = seg_a
        x3, y3, x4, y4 = seg_b
        
        da = np.array([x2 - x1, y2 - y1])
        db = np.array([x4 - x3, y4 - y3])
        det = da[0] * db[1] - da[1] * db[0]
        
        if abs(det) < 1e-6:
            return 0.0, None
        
        s = ((x3 - x1) * db[1] - (y3 - y1) * db[0]) / det
        t = ((x3 - x1) * da[1] - (y3 - y1) * da[0]) / det
        
        if 0 <= s <= 1 and 0 <= t <= 1:
            pt = np.array([x1 + s * da[0], y1 + s * da[1]])
            severity = 0.5 * (s + t)
            return severity, pt
        
        return 0.0, None
    
    def _log_metrics(self):
        """记录群体级metrics"""
        active = sum(1 for r in self.robots.values() if not r.reached_goal)
        tensions = [r.tether_tension for r in self.robots.values()]
        avg_tension = np.mean(tensions) if tensions else 0.0
        max_tension = np.max(tensions) if tensions else 0.0

        elapsed = self.current_time
        all_at_goal = all(r.reached_goal for r in self.robots.values())

        # 纠缠对
        pairs = list(set(
            (e.robot_a, e.robot_b) for e in self.entanglement_events
        ))
        pair_strs = [f"{a}-{b}" for a, b in pairs]

        # 确保数据有效
        timestamp = round(self.current_time, 10)  # 高精度防止浮点误差

        row = [
            timestamp, active, self.total_entanglements,
            len(self.entanglement_events), float(avg_tension), float(max_tension),
            float(elapsed if all_at_goal else -1), bool(all_at_goal), ';'.join(pair_strs)
        ]

        try:
            self.metrics_writer.writerow(row)
            self.metrics_file.flush()  # 强制刷新，防止缓冲导致的数据丢失
        except Exception as e:
            print(f"⚠️  CSV写入错误: {e}")
    
    def _log_trajectories(self, step: int):
        """记录每个机器人的轨迹"""
        for robot_id, robot in self.robots.items():
            try:
                self.traj_writers[robot_id].writerow([
                    round(self.current_time, 10),
                    float(robot.pose[0]), float(robot.pose[1]), float(robot.pose[2]),
                    float(robot.velocity[0]) if np.linalg.norm(robot.velocity) > 0 else 0.0,
                    float(robot.velocity[1]) if np.linalg.norm(robot.velocity) > 0 else 0.0,
                    float(robot.tether_tension), float(robot.tether_length),
                    robot.current_yielding.value,
                    float(robot.distance_to_goal()),
                    float(self.controllers[robot_id].total_reward)
                ])
                # 每10步刷新一次
                if step % 10 == 0:
                    self.traj_files[robot_id].flush()
            except Exception as e:
                print(f"⚠️  轨迹写入错误 (robot {robot_id}): {e}")
    
    def _finalize(self):
        """结束仿真，保存数据"""
        end_time = time.time()
        self.metadata['end_time'] = datetime.now().isoformat()
        self.metadata['total_duration'] = end_time - self.start_time if self.start_time else 0
        
        # 关闭所有文件
        self.metrics_file.close()
        for f in self.traj_files.values():
            f.close()
        self.crossing_file.close()
        self.yielding_file.close()
        
        # 保存元数据
        with open(self.log_dir / 'metadata.json', 'w') as f:
            json.dump(self.metadata, f, indent=2)
        
        # 保存摘要
        self._save_summary()
        
        print(f"\n✅ 仿真完成！数据已保存到: {self.log_dir}")
    
    def _save_summary(self):
        """保存实验摘要"""
        all_reached = all(r.reached_goal for r in self.robots.values())
        total_entanglements = self.total_entanglements
        duration = self.current_time
        
        avg_tension_final = np.mean([r.tether_tension for r in self.robots.values()])
        max_tension_final = np.max([r.tether_tension for r in self.robots.values()])
        
        summary = f"""
========================================
  Swarm Simulation Summary
========================================
Scenario: {self.scenario_name}
Mode: {self.mode}
Robots: {self.num_robots}

Results:
  All reached goal: {all_reached}
  Total entanglements: {total_entanglements}
  Simulation duration: {duration:.1f}s
  Final avg tension: {avg_tension_final:.1f}N
  Final max tension: {max_tension_final:.1f}N

Yielding events: {len(self.yielding_decisions)}
Crossing events: {len(self.entanglement_events)}

Data saved to: {self.log_dir}
========================================
"""
        print(summary)
        
        with open(self.log_dir / 'summary.txt', 'w') as f:
            f.write(summary)

# ==================== Scenario Definitions ====================

class BottleneckScenario:
    """Scenario A: Swarm Bottleneck - 狭窄走廊"""
    def __init__(self, num_robots: int):
        self.name = "bottleneck"
        self.num_robots = num_robots
        self.robot_start_poses = []
        self.robot_goals = []
        
        random.seed(42)
        for i in range(num_robots):
            # 入口区域（左侧）
            sx = -7.0 + random.uniform(-1.0, 1.0)
            sy = random.uniform(-1.5, 1.5)
            syaw = random.uniform(-0.3, 0.3)
            
            # 出口区域（右侧）
            gx = 7.0 + random.uniform(-1.0, 1.0)
            gy = random.uniform(-1.5, 1.5)
            
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))

class CrossingPathsScenario:
    """Scenario B: Crossing Paths - 交叉路径 X-pattern"""
    def __init__(self, num_robots: int):
        self.name = "crossing"
        self.num_robots = num_robots
        self.robot_start_poses = []
        self.robot_goals = []
        
        corners = [(-8, 8), (8, 8), (-8, -8), (8, -8)]
        opposites = [(8, -8), (-8, -8), (8, 8), (-8, 8)]
        
        for i in range(min(num_robots, 4)):
            sx, sy = corners[i]
            gx, gy = opposites[i]
            syaw = math.atan2(gy - sy, gx - sx)
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))
        
        # 额外机器人在中心附近
        for i in range(4, num_robots):
            sx = random.uniform(-5, 5)
            sy = random.uniform(-5, 5)
            gx = random.uniform(-8, 8)
            gy = random.uniform(-8, 8)
            syaw = random.uniform(-math.pi, math.pi)
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))

class ExpansionScenario:
    """Scenario C: Swarm Expansion - 从中心锚点向外扩展"""
    def __init__(self, num_robots: int):
        self.name = "expansion"
        self.num_robots = num_robots
        self.robot_start_poses = []
        self.robot_goals = []
        
        random.seed(42)
        anchor_x, anchor_y = 0.0, 0.0
        
        for i in range(num_robots):
            angle = (2 * math.pi / num_robots) * i + random.uniform(-0.2, 0.2)
            
            # 起始：锚点附近
            start_r = random.uniform(0.5, 1.5)
            sx = anchor_x + start_r * math.cos(angle)
            sy = anchor_y + start_r * math.sin(angle)
            syaw = angle
            
            # 目标：向外扩展（有障碍）
            goal_r = random.uniform(8, 12)
            gx = anchor_x + goal_r * math.cos(angle)
            gy = anchor_y + goal_r * math.sin(angle)
            
            self.robot_start_poses.append((sx, sy, syaw))
            self.robot_goals.append((gx, gy))

# ==================== CLI ====================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Swarm Robotics Simulator for npj Robotics Paper')
    parser.add_argument('--scenario', type=str, required=True,
                       choices=['bottleneck', 'crossing', 'expansion'],
                       help='Scenario name')
    parser.add_argument('--mode', type=str, required=True,
                       choices=['baseline', 'cooperative'],
                       help='Experiment mode')
    parser.add_argument('--num-robots', type=int, default=5,
                       help='Number of robots (default: 5)')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed for reproducibility')
    
    args = parser.parse_args()
    
    # 设置随机种子
    random.seed(args.seed)
    np.random.seed(args.seed)
    
    # 创建并运行仿真
    sim = SwarmSimulation(args.scenario, args.num_robots, args.mode)
    sim.run()

if __name__ == '__main__':
    main()
