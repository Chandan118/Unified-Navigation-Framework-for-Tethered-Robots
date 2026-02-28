#!/usr/bin/env python3
"""
Fuzzy Logic Controller Node for Hybrid Navigation
Implements the Fuzzy Logic Controller described in Section 5.2 of the paper
with 125 rules for navigation control (ROS 2 version)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import std_msgs.msg
from hybrid_navigation_msgs.msg import TetherStatus, SceneComplexity
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyNavigationController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safety_distance', 1.0)
        
        self.linear_speed_base = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        
        # State variables
        self.min_obstacle_distance = float('inf')
        self.tether_tension = 0.0
        self.tether_length = 0.0
        self.scene_complexity = 0.5
        self.goal_angle = 0.0
        
        # Initialize Fuzzy Logic System
        self.setup_fuzzy_system()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(TetherStatus, '/tether_status', self.tether_callback, 10)
        self.create_subscription(SceneComplexity, '/scene_complexity', self.scene_callback, 10)
        self.create_subscription(std_msgs.msg.Float32, '/goal_angle', self.goal_angle_callback, 10)
        
        # Control loop timer
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.compute_control)
        
        self.get_logger().info("Fuzzy Navigation Controller initialized (ROS 2)")
    
    def setup_fuzzy_system(self):
        """Setup fuzzy logic membership functions and rules"""
        
        # ===== ANTECEDENTS (Inputs) =====
        # Distance to nearest obstacle (0-5 meters)
        self.distance = ctrl.Antecedent(np.arange(0, 5.1, 0.1), 'distance')
        self.distance['very_near'] = fuzz.trimf(self.distance.universe, [0, 0, 0.5])
        self.distance['near'] = fuzz.trimf(self.distance.universe, [0.3, 0.7, 1.2])
        self.distance['medium'] = fuzz.trimf(self.distance.universe, [0.8, 1.5, 2.5])
        self.distance['far'] = fuzz.trimf(self.distance.universe, [2.0, 3.0, 5.0])
        self.distance['very_far'] = fuzz.trimf(self.distance.universe, [4.0, 5.0, 5.0])
        
        # Tether tension (0-50 Newtons - simulated)
        self.tension = ctrl.Antecedent(np.arange(0, 50.1, 1), 'tension')
        self.tension['low'] = fuzz.trimf(self.tension.universe, [0, 0, 10])
        self.tension['safe'] = fuzz.trapmf(self.tension.universe, [5, 10, 20, 25])
        self.tension['high'] = fuzz.trapmf(self.tension.universe, [20, 30, 40, 45])
        self.tension['critical'] = fuzz.trimf(self.tension.universe, [40, 50, 50])
        
        # Goal angle error (-180 to 180 degrees)
        self.goal_error = ctrl.Antecedent(np.arange(-180, 181, 1), 'goal_error')
        self.goal_error['large_left'] = fuzz.trimf(self.goal_error.universe, [-180, -180, -90])
        self.goal_error['left'] = fuzz.trimf(self.goal_error.universe, [-120, -45, -10])
        self.goal_error['straight'] = fuzz.trimf(self.goal_error.universe, [-15, 0, 15])
        self.goal_error['right'] = fuzz.trimf(self.goal_error.universe, [10, 45, 120])
        self.goal_error['large_right'] = fuzz.trimf(self.goal_error.universe, [90, 180, 180])
        
        # Scene complexity (0-1, from CNN)
        self.complexity = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'complexity')
        self.complexity['simple'] = fuzz.trimf(self.complexity.universe, [0, 0, 0.4])
        self.complexity['moderate'] = fuzz.trimf(self.complexity.universe, [0.3, 0.5, 0.7])
        self.complexity['complex'] = fuzz.trimf(self.complexity.universe, [0.6, 1.0, 1.0])
        
        # ===== CONSEQUENTS (Outputs) =====
        # Linear velocity (0-1 m/s)
        self.linear_vel = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'linear_vel')
        self.linear_vel['stop'] = fuzz.trimf(self.linear_vel.universe, [0, 0, 0.1])
        self.linear_vel['very_slow'] = fuzz.trimf(self.linear_vel.universe, [0, 0.15, 0.3])
        self.linear_vel['slow'] = fuzz.trimf(self.linear_vel.universe, [0.2, 0.35, 0.5])
        self.linear_vel['medium'] = fuzz.trimf(self.linear_vel.universe, [0.4, 0.6, 0.8])
        self.linear_vel['fast'] = fuzz.trimf(self.linear_vel.universe, [0.7, 1.0, 1.0])
        
        # Angular velocity (-1 to 1 rad/s)
        self.angular_vel = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'angular_vel')
        self.angular_vel['sharp_left'] = fuzz.trimf(self.angular_vel.universe, [-1, -1, -0.6])
        self.angular_vel['left'] = fuzz.trimf(self.angular_vel.universe, [-0.8, -0.4, -0.1])
        self.angular_vel['slight_left'] = fuzz.trimf(self.angular_vel.universe, [-0.3, -0.15, 0])
        self.angular_vel['straight'] = fuzz.trimf(self.angular_vel.universe, [-0.1, 0, 0.1])
        self.angular_vel['slight_right'] = fuzz.trimf(self.angular_vel.universe, [0, 0.15, 0.3])
        self.angular_vel['right'] = fuzz.trimf(self.angular_vel.universe, [0.1, 0.4, 0.8])
        self.angular_vel['sharp_right'] = fuzz.trimf(self.angular_vel.universe, [0.6, 1, 1])
        
        # ===== RULES =====
        # Implementing key rules from the paper's 125-rule system
        rules = []
        
        # SAFETY RULES (Highest Priority)
        # Rule 1: If obstacle very near, STOP
        rules.append(ctrl.Rule(self.distance['very_near'], 
                              (self.linear_vel['stop'], self.angular_vel['sharp_left'])))
        
        # Rule 2: If tether tension critical, slow down and turn toward base
        rules.append(ctrl.Rule(self.tension['critical'], 
                              (self.linear_vel['stop'], self.angular_vel['sharp_left'])))
        
        # OBSTACLE AVOIDANCE RULES
        # Rule 3: Near obstacle + low tension -> slow and avoid
        rules.append(ctrl.Rule(self.distance['near'] & self.tension['low'], 
                              (self.linear_vel['very_slow'], self.angular_vel['left'])))
        
        # Rule 4: Near obstacle + safe tension -> moderate avoidance
        rules.append(ctrl.Rule(self.distance['near'] & self.tension['safe'], 
                              (self.linear_vel['slow'], self.angular_vel['slight_left'])))
        
        # GOAL-SEEKING RULES
        # Rule 5: Far from obstacle + straight to goal + low complexity -> fast
        rules.append(ctrl.Rule(self.distance['far'] & self.goal_error['straight'] & self.complexity['simple'],
                              (self.linear_vel['fast'], self.angular_vel['straight'])))
        
        # Rule 6: Far + need to turn left -> medium speed, turn left
        rules.append(ctrl.Rule(self.distance['far'] & self.goal_error['left'],
                              (self.linear_vel['medium'], self.angular_vel['left'])))
        
        # Rule 7: Far + need to turn right -> medium speed, turn right
        rules.append(ctrl.Rule(self.distance['far'] & self.goal_error['right'],
                              (self.linear_vel['medium'], self.angular_vel['right'])))
        
        # TETHER-AWARE RULES
        # Rule 8: Medium distance + high tension -> slow down
        rules.append(ctrl.Rule(self.distance['medium'] & self.tension['high'],
                              (self.linear_vel['very_slow'], self.angular_vel['slight_left'])))
        
        # Rule 9: Far + safe tension + heading to goal -> maintain speed
        rules.append(ctrl.Rule(self.distance['far'] & self.tension['safe'] & self.goal_error['straight'],
                              (self.linear_vel['medium'], self.angular_vel['straight'])))
        
        # COMPLEXITY-BASED RULES
        # Rule 10: Complex scene + near obstacle -> very cautious
        rules.append(ctrl.Rule(self.complexity['complex'] & self.distance['near'],
                              (self.linear_vel['very_slow'], self.angular_vel['slight_left'])))
        
        # Rule 11: Simple scene + medium distance -> faster
        rules.append(ctrl.Rule(self.complexity['simple'] & self.distance['medium'],
                              (self.linear_vel['medium'], self.angular_vel['straight'])))
        
        # Rule 12: Moderate complexity + medium distance + safe tension
        rules.append(ctrl.Rule(self.complexity['moderate'] & self.distance['medium'] & self.tension['safe'],
                              (self.linear_vel['slow'], self.angular_vel['slight_left'])))
        
        # Additional rules for comprehensive coverage (simplified from 125)
        # Rules 13-25: Various combinations for robust behavior
        rules.append(ctrl.Rule(self.distance['very_far'] & self.tension['low'],
                              (self.linear_vel['fast'], self.angular_vel['straight'])))
        
        rules.append(ctrl.Rule(self.goal_error['large_left'] & self.distance['medium'],
                              (self.linear_vel['slow'], self.angular_vel['sharp_left'])))
        
        rules.append(ctrl.Rule(self.goal_error['large_right'] & self.distance['medium'],
                              (self.linear_vel['slow'], self.angular_vel['sharp_right'])))
        
        # Create control system
        self.control_system = ctrl.ControlSystem(rules)
        self.controller = ctrl.ControlSystemSimulation(self.control_system)
        
        self.get_logger().info(f"Fuzzy logic system initialized with {len(rules)} rules")
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        # Filter invalid readings
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            self.min_obstacle_distance = min(valid_ranges)
        else:
            self.min_obstacle_distance = msg.range_max
    
    def tether_callback(self, msg):
        """Process tether status"""
        self.tether_tension = msg.tension
        self.tether_length = msg.length
    
    def scene_callback(self, msg):
        """Process scene complexity from CNN"""
        self.scene_complexity = msg.complexity_score
    
    def goal_angle_callback(self, msg):
        """Process goal direction"""
        self.goal_angle = msg.data
    
    def compute_control(self):
        """Compute control commands using fuzzy logic"""
        try:
            # Set inputs
            self.controller.input['distance'] = np.clip(self.min_obstacle_distance, 0, 5.0)
            self.controller.input['tension'] = np.clip(self.tether_tension, 0, 50.0)
            self.controller.input['goal_error'] = np.clip(self.goal_angle, -180, 180)
            self.controller.input['complexity'] = np.clip(self.scene_complexity, 0, 1.0)
            
            # Compute output
            self.controller.compute()
            
            # Get outputs
            linear_vel = float(self.controller.output['linear_vel'])
            angular_vel = float(self.controller.output['angular_vel'])
            
            # Create and publish command
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            self.get_logger().debug(f"Control: lin={linear_vel:.2f}, ang={angular_vel:.2f} | dist={self.min_obstacle_distance:.2f}, tension={self.tether_tension:.2f}")
            
        except Exception as e:
            self.get_logger().warn(f"Fuzzy control computation failed: {str(e)}")
            # Publish stop command on error
            self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = FuzzyNavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
