#!/usr/bin/env python3
"""
Enhanced Fuzzy Controller for Swarm Coordination
==================================================
扩展原有的 fuzzy controller，添加 swarm coordination 输入

新增输入：
- nearby_robot_tension: 邻近机器人的 tether 张力
- crossing_severity: 交叉严重程度
- priority_ratio: 与对方机器人的优先级比

新增输出：
- yielding_adjustment: 专门针对协调的速度调整 (-1 到 1)

集成到现有控制循环：
- velocity_adj 和 steering_adj 现在包括协同调整
"""

import rclpy
from rclpy.node import Node
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 导入新的消息类型
try:
    from tethered_navigation.msg import YieldingDecision
except ImportError:
    YieldingDecision = None


class EnhancedFuzzyController(Node):
    """
    增强型模糊控制器，支持 swarm coordination
    """
    
    def __init__(self):
        super().__init__('enhanced_fuzzy_controller')
        
        # QoS
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- 1. 原有模糊变量 ---
        # Obstacle distance: 0-5 meters
        self.obstacle_distance = ctrl.Antecedent(np.arange(0, 5.1, 0.1), 'obstacle_distance')
        # Tether tension: 0-50 N
        self.tether_tension = ctrl.Antecedent(np.arange(0, 51, 1), 'tether_tension')
        
        # --- 2. 新增协同模糊变量 ---
        # Nearby robot tension (其他机器人 tether 张力)
        self.neighbor_tension = ctrl.Antecedent(np.arange(0, 51, 1), 'neighbor_tension')
        # Crossing severity (交叉严重程度 0-1)
        self.crossing_severity = ctrl.Antecedent(np.arange(0, 1.1, 0.1), 'crossing_severity')
        # Priority ratio (我方/对方, 0-2)
        self.priority_ratio = ctrl.Antecedent(np.arange(0, 2.1, 0.1), 'priority_ratio')
        
        # --- 3. 输出 ---
        # 原有输出
        self.velocity_adj = ctrl.Consequent(np.arange(-1.0, 1.1, 0.1), 'velocity_adj')
        self.steering_adj = ctrl.Consequent(np.arange(-1.0, 1.1, 0.1), 'steering_adj')
        # 新增：协同调整输出
        self.yielding_adj = ctrl.Consequent(np.arange(-1.0, 1.1, 0.1), 'yielding_adj')
        
        # --- 4. 定义隶属函数 ---
        self._define_membership_functions()
        
        # --- 5. 定义模糊规则 ---
        rules = self._define_rules()
        
        # --- 6. 创建控制系统 ---
        self.control_system = ctrl.ControlSystem(rules)
        self.controller = ctrl.ControlSystemSimulation(self.control_system)
        
        # --- ROS 2 通信 ---
        # 订阅主 navigator 的模糊输入
        self.fuzzy_input_sub = self.create_subscription(
            Float32MultiArray,
            '/fuzzy_input',  # 全局 topic（每个机器人 namespace 下）
            self.fuzzy_input_callback,
            10
        )
        
        # 订阅 swarm coordinator 的 yielding decision
        if YieldingDecision:
            self.yielding_sub = self.create_subscription(
                YieldingDecision,
                '/yielding_decisions',
                self.yielding_callback,
                qos_reliable
            )
        
        # 发布调整��的控制量
        self.adjustment_pub = self.create_publisher(Twist, '/fuzzy_adjustment', 10)
        
        # 状态
        self.current_yielding_action = 'maintain'  # 从 coordinator 接收
        self.current_crossing_severity = 0.0
        self.current_neighbor_tension = 30.0  # 默认中等
        self.current_priority_ratio = 1.0
        
        self.get_logger().info('Enhanced Fuzzy Controller 已启动')
    
    def _define_membership_functions(self):
        """定义所有模糊变量的隶属函数"""
        
        # === Obstacle Distance ===
        self.obstacle_distance['near'] = fuzz.trimf(self.obstacle_distance.universe, [0, 0, 1.0])
        self.obstacle_distance['medium'] = fuzz.trimf(self.obstacle_distance.universe, [0.5, 1.5, 2.5])
        self.obstacle_distance['far'] = fuzz.trimf(self.obstacle_distance.universe, [2.0, 5.0, 5.0])
        
        # === Tether Tension ===
        self.tether_tension['low'] = fuzz.trimf(self.tether_tension.universe, [0, 0, 20])
        self.tether_tension['medium'] = fuzz.trimf(self.tether_tension.universe, [10, 30, 50])
        self.tether_tension['high'] = fuzz.trimf(self.tether_tension.universe, [30, 50, 50])
        
        # === Neighbor Tension ===
        self.neighbor_tension['low'] = fuzz.trimf(self.neighbor_tension.universe, [0, 0, 20])
        self.neighbor_tension['medium'] = fuzz.trimf(self.neighbor_tension.universe, [10, 30, 50])
        self.neighbor_tension['high'] = fuzz.trimf(self.neighbor_tension.universe, [30, 50, 50])
        
        # === Crossing Severity ===
        self.crossing_severity['none'] = fuzz.trimf(self.crossing_severity.universe, [0, 0, 0.3])
        self.crossing_severity['moderate'] = fuzz.trimf(self.crossing_severity.universe, [0.2, 0.5, 0.8])
        self.crossing_severity['severe'] = fuzz.trimf(self.crossing_severity.universe, [0.6, 1.0, 1.0])
        
        # === Priority Ratio ===
        # 比率 < 0.8: 我方优先级低（应该让行）
        # 比率 0.8-1.2: 优先级相近
        # 比率 > 1.2: 我方优先级高（对方应该让行）
        self.priority_ratio['lower'] = fuzz.trimf(self.priority_ratio.universe, [0, 0.5, 0.8])
        self.priority_ratio['equal'] = fuzz.trimf(self.priority_ratio.universe, [0.6, 1.0, 1.4])
        self.priority_ratio['higher'] = fuzz.trimf(self.priority_ratio.universe, [1.2, 1.5, 2.0])
        
        # === Velocity Adjustment ===
        self.velocity_adj['reduce'] = fuzz.trimf(self.velocity_adj.universe, [-1.0, -1.0, 0])
        self.velocity_adj['maintain'] = fuzz.trimf(self.velocity_adj.universe, [-0.2, 0, 0.2])
        self.velocity_adj['increase'] = fuzz.trimf(self.velocity_adj.universe, [0, 1.0, 1.0])
        
        # === Steering Adjustment ===
        self.steering_adj['left'] = fuzz.trimf(self.steering_adj.universe, [-1.0, -1.0, 0])
        self.steering_adj['none'] = fuzz.trimf(self.steering_adj.universe, [-0.2, 0, 0.2])
        self.steering_adj['right'] = fuzz.trimf(self.steering_adj.universe, [0, 1.0, 1.0])
        
        # === Yielding Adjustment (协同专用) ===
        # 负值表示减速让行，正值表示加速通过
        self.yielding_adj['yield'] = fuzz.trimf(self.yielding_adj.universe, [-1.0, -1.0, -0.3])
        self.yielding_adj['neutral'] = fuzz.trimf(self.yielding_adj.universe, [-0.4, 0, 0.4])
        self.yielding_adj['go'] = fuzz.trimf(self.yielding_adj.universe, [0.3, 1.0, 1.0])
    
    def _define_rules(self):
        """定义模糊规则库"""
        rules = []
        
        # === 原有规则（障碍物避障 + tether 管理） ===
        # 如果障碍物近且张力高 -> 减速并左转
        rules.append(ctrl.Rule(
            self.obstacle_distance['near'] & self.tether_tension['high'],
            (self.velocity_adj['reduce'], self.steering_adj['left'])
        ))
        
        # 如果障碍物近且张力低 -> 减速���右转
        rules.append(ctrl.Rule(
            self.obstacle_distance['near'] & self.tether_tension['low'],
            (self.velocity_adj['reduce'], self.steering_adj['right'])
        ))
        
        # 障碍物中等距离 -> 保持速度和方向
        rules.append(ctrl.Rule(
            self.obstacle_distance['medium'],
            (self.velocity_adj['maintain'], self.steering_adj['none'])
        ))
        
        # 障碍物远 -> 加速直行
        rules.append(ctrl.Rule(
            self.obstacle_distance['far'],
            (self.velocity_adj['increase'], self.steering_adj['none'])
        ))
        
        # === Swarm Coordination 规则（新增）===
        
        # 规则 S1: 如果交叉严重且我方优先级低 -> 主动让行（yield）
        rules.append(ctrl.Rule(
            self.crossing_severity['severe'] & self.priority_ratio['lower'],
            self.yielding_adj['yield']
        ))
        
        # 规则 S2: 如果交叉严重且我方优先级高 -> 保持通过（go），轻微调整方向
        rules.append(ctrl.Rule(
            self.crossing_severity['severe'] & self.priority_ratio['higher'],
            self.yielding_adj['go']
        ))
        
        # 规则 S3: 如果交叉中等且优先级相近 -> 协商（neutral），缓慢通过
        rules.append(ctrl.Rule(
            self.crossing_severity['moderate'] & self.priority_ratio['equal'],
            self.yielding_adj['neutral']
        ))
        
        # 规则 S4: 如果对方张力很高（紧急），我方让行（即使优先级不低）
        rules.append(ctrl.Rule(
            self.neighbor_tension['high'] & self.crossing_severity['moderate'],
            self.yielding_adj['yield']
        ))
        
        # 规则 S5: 如果对方张力低且我方张力高 -> 我方可安全通过（go）
        rules.append(ctrl.Rule(
            self.neighbor_tension['low'] & self.tether_tension['high'] & 
            self.priority_ratio['higher'],
            self.yielding_adj['go']
        ))
        
        return rules
    
    def fuzzy_input_callback(self, msg: Float32MultiArray):
        """
        接收来自 navigator 的模糊输入：[obstacle_distance, tether_tension]
        """
        if len(msg.data) < 2:
            self.get_logger().warn("模糊输入数据不足")
            return
        
        obstacle_dist = msg.data[0]
        tether_tension = msg.data[1]
        
        # 输入到控制器
        self.controller.input['obstacle_distance'] = obstacle_dist
        self.controller.input['tether_tension'] = tether_tension
        
        # 输入协同变量
        self.controller.input['neighbor_tension'] = self.current_neighbor_tension
        self.controller.input['crossing_severity'] = self.current_crossing_severity
        self.controller.input['priority_ratio'] = self.current_priority_ratio
        
        # 计算
        try:
            self.controller.compute()
            
            # 获取输出
            velocity_adj = self.controller.output['velocity_adj']
            steering_adj = self.controller.output['steering_adj']
            yielding_adj = self.controller.output['yielding_adj']
            
            # 融合：基础调整 + 协同调整
            # yielding_adj 主要影响 velocity_adj
            final_velocity_adj = 0.7 * velocity_adj + 0.3 * yielding_adj
            
            # 如果主动让行，可能需要额外转向（避免交叉）
            final_steering_adj = steering_adj
            if yielding_adj < -0.5:
                # 让行时可能需要轻微转向避开对方 tether
                # 这个转向方向需要根据 crossing 位置决定，这里简化
                final_steering_adj += -0.2 if np.random.rand() > 0.5 else 0.2
            
            # 发布 Twist 调整消息
            adjustment = Twist()
            adjustment.linear.x = float(final_velocity_adj)
            adjustment.angular.z = float(final_steering_adj)
            self.adjustment_pub.publish(adjustment)
            
        except Exception as e:
            self.get_logger().warn(f'模糊推理失败: {e}')
    
    def yielding_callback(self, msg: YieldingDecision):
        """
        接收 swarm coordinator 的 yielding 决策
        """
        if msg.requesting_robot_id != self.get_namespace():
            return  # 不是发给我的
        
        self.current_yielding_action = msg.suggested_action
        self.current_crossing_severity = msg.crossing_severity
        self.current_priority_ratio = msg.priority_comparison
        
        self.get_logger().debug(
            f'收到 yielding 决策: action={msg.suggested_action}, '
            f'severity={msg.crossing_severity:.2f}'
        )
    
    def get_namespace(self) -> str:
        """获取当前机器人的 namespace"""
        # 从 node name 推断，例如 /robot_1/swarm_coordinator -> robot_1
        full_name = self.get_fully_qualified_name()
        parts = full_name.split('/')
        if len(parts) >= 2 and parts[1].startswith('robot_'):
            return parts[1]
        return 'robot_1'


def main(args=None):
    rclpy.init(args=args)
    
    controller = EnhancedFuzzyController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
