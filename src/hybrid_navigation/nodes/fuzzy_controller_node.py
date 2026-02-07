#!/usr/bin/env python3
"""
Fuzzy Logic Controller Node for Hybrid Navigation
Implements the Fuzzy Logic Controller described in Section 5.2 of the paper
with 125 rules for navigation control
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import std_msgs.msg
from hybrid_navigation.msg import TetherStatus, SceneComplexity
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyNavigationController:
    def __init__(self):
        rospy.init_node('fuzzy_controller_node', anonymous=False)
        
        # Parameters
        self.linear_speed_base = rospy.get_param('~linear_speed', 0.5)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.safety_distance = rospy.get_param('~safety_distance', 1.0)
        
        # State variables
        self.min_obstacle_distance = float('inf')
        self.tether_tension = 0.0
        self.tether_length = 0.0
        self.scene_complexity = 0.5
        self.goal_angle = 0.0
        
        # Initialize Fuzzy Logic System
        self.setup_fuzzy_system()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/tether_status', TetherStatus, self.tether_callback)
        rospy.Subscriber('/scene_complexity', SceneComplexity, self.scene_callback)
        rospy.Subscriber('/goal_angle', std_msgs.msg.Float32, self.goal_angle_callback)
        
        # Control loop timer
        self.control_rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Fuzzy Navigation Controller initialized")
    
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
        
        # ===== ADDITIONAL BROAD COVERAGE RULES =====
        # Ensure that if we are far from obstacles, we generally move forward
        rules.append(ctrl.Rule(self.distance['far'],
                              (self.linear_vel['medium'], self.angular_vel['straight'])))
        
        # Ensure that if tension is safe/low, we generally move forward - BUT ONLY IF SAFE DISTANCE
        # We replace the unconditional tension rules with distance-conditioned ones
        rules.append(ctrl.Rule(self.tension['safe'] & self.distance['medium'],
                              (self.linear_vel['slow'], self.angular_vel['straight'])))

        rules.append(ctrl.Rule(self.tension['low'] & self.distance['medium'],
                              (self.linear_vel['medium'], self.angular_vel['straight'])))
        
        # If goal is largely to the side, turn
        rules.append(ctrl.Rule(self.goal_error['large_left'],
                              (self.linear_vel['very_slow'], self.angular_vel['sharp_left'])))
        
        rules.append(ctrl.Rule(self.goal_error['large_right'],
                              (self.linear_vel['very_slow'], self.angular_vel['sharp_right'])))

        # Create control system
        self.control_system = ctrl.ControlSystem(rules)
        self.controller = ctrl.ControlSystemSimulation(self.control_system)
        
        rospy.loginfo("Fuzzy logic system initialized with %d rules", len(rules))
    
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
        cmd = Twist()
        try:
            # Set inputs
            self.controller.input['distance'] = np.clip(self.min_obstacle_distance, 0, 5.0)
            self.controller.input['tension'] = np.clip(self.tether_tension, 0, 50.0)
            self.controller.input['goal_error'] = np.clip(self.goal_angle, -180, 180)
            self.controller.input['complexity'] = np.clip(self.scene_complexity, 0, 1.0)
            
            # Compute output
            self.controller.compute()
            
            # Get outputs
            linear_vel = self.controller.output['linear_vel']
            angular_vel = self.controller.output['angular_vel']
            
            # Create command
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            
            rospy.logdebug("Control: lin=%.2f, ang=%.2f | dist=%.2f, tension=%.2f", 
                          linear_vel, angular_vel, self.min_obstacle_distance, self.tether_tension)
            
        except Exception as e:
            rospy.logwarn_throttle(1, "Fuzzy control computation failed (using safety fallback): %s", str(e))
            # Fallback behavior: Slow forward creeping if safe, otherwise stop
            if self.min_obstacle_distance > 0.5 and self.tether_tension < 20:
                cmd.linear.x = 0.1  # Creep forward
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        # Publish command (either computed or fallback)
        self.cmd_vel_pub.publish(cmd)
    
    def run(self):
        """Main control loop"""
        rospy.loginfo("Fuzzy controller running...")
        
        while not rospy.is_shutdown():
            self.compute_control()
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = FuzzyNavigationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
