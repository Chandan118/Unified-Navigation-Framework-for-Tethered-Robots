#!/usr/bin/env python3
"""
Hybrid Planner Node - Finite State Machine for Navigation (ROS 2 version)
Implements Motion-to-Goal, Enhanced Bug, and Tether Recovery states
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from hybrid_navigation_msgs.msg import TetherStatus, SceneComplexity

class NavigationState(Enum):
    MOTION_TO_GOAL = 1
    ENHANCED_BUG = 2
    TETHER_RECOVERY = 3
    STUCK = 4

class HybridPlannerNode(Node):
    def __init__(self):
        super().__init__('hybrid_planner_node')
        
        # Parameters
        self.declare_parameter('obstacle_threshold', 1.5)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('wall_follow_distance', 0.7)
        self.declare_parameter('max_tether_usage', 0.95)
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').get_parameter_value().double_value
        self.max_tether_usage = self.get_parameter('max_tether_usage').get_parameter_value().double_value
        
        # State
        self.current_state = NavigationState.MOTION_TO_GOAL
        self.goal = None
        self.robot_pose = None
        self.robot_orientation = 0.0
        
        # Sensor data
        self.scan_data = None
        self.tether_status = None
        self.scene_complexity = 0.5
        
        # Bug algorithm state
        self.hit_point = None
        self.leave_point = None
        self.wall_following_direction = 1  # 1 for left, -1 for right
        
        # Publishers
        self.goal_angle_pub = self.create_publisher(Float32, '/goal_angle', 10)
        self.state_pub = self.create_publisher(String, '/planner_state', 10)
        
        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(TetherStatus, '/tether_status', self.tether_callback, 10)
        self.create_subscription(SceneComplexity, '/scene_complexity', self.scene_callback, 10)
        
        # Control loop
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info("Hybrid Planner initialized (ROS 2)")
    
    def goal_callback(self, msg):
        """Receive new navigation goal"""
        self.goal = msg.pose.position
        self.current_state = NavigationState.MOTION_TO_GOAL
        self.get_logger().info(f"New goal received: ({self.goal.x:.2f}, {self.goal.y:.2f})")
    
    def odom_callback(self, msg):
        """Update robot pose"""
        self.robot_pose = msg.pose.pose.position
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg):
        """Store laser scan data"""
        self.scan_data = msg
    
    def tether_callback(self, msg):
        """Store tether status"""
        self.tether_status = msg
    
    def scene_callback(self, msg):
        """Store scene complexity"""
        self.scene_complexity = msg.complexity_score
    
    def get_distance_to_goal(self):
        """Calculate Euclidean distance to goal"""
        if self.goal is None or self.robot_pose is None:
            return float('inf')
        
        dx = self.goal.x - self.robot_pose.x
        dy = self.goal.y - self.robot_pose.y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_goal(self):
        """Calculate angle to goal in degrees"""
        if self.goal is None or self.robot_pose is None:
            return 0.0
        
        dx = self.goal.x - self.robot_pose.x
        dy = self.goal.y - self.robot_pose.y
        goal_angle = math.atan2(dy, dx)
        
        # Calculate error from current orientation
        angle_error = goal_angle - self.robot_orientation
        
        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Convert to degrees
        return math.degrees(angle_error)
    
    def get_min_obstacle_distance(self):
        """Get minimum distance to obstacle from laser scan"""
        if self.scan_data is None:
            return float('inf')
        
        valid_ranges = [r for r in self.scan_data.ranges 
                       if self.scan_data.range_min < r < self.scan_data.range_max]
        
        return min(valid_ranges) if valid_ranges else float('inf')
    
    def is_path_clear(self):
        """Check if path to goal is clear of obstacles"""
        min_distance = self.get_min_obstacle_distance()
        return min_distance > self.obstacle_threshold
    
    def is_tether_critical(self):
        """Check if tether is in critical state"""
        if self.tether_status is None:
            return False
        
        length_critical = (self.tether_status.length / self.tether_status.max_length) > self.max_tether_usage
        snag_detected = self.tether_status.snag_detected
        tension_critical = self.tether_status.tension > 40.0
        
        return length_critical or snag_detected or tension_critical
    
    def state_transition(self):
        """Finite State Machine transitions"""
        
        if self.goal is None or self.robot_pose is None:
            return
        
        # Check if goal reached
        if self.get_distance_to_goal() < self.goal_tolerance:
            self.get_logger().info("Goal reached!")
            self.goal = None
            self.current_state = NavigationState.MOTION_TO_GOAL
            return
        
        # Priority 1: Tether Recovery (highest priority)
        if self.is_tether_critical():
            if self.current_state != NavigationState.TETHER_RECOVERY:
                self.get_logger().warn("Tether critical! Entering recovery mode")
                self.current_state = NavigationState.TETHER_RECOVERY
            return
        
        # Priority 2: Enhanced Bug (obstacle avoidance)
        if not self.is_path_clear():
            if self.current_state == NavigationState.MOTION_TO_GOAL:
                self.get_logger().info("Obstacle detected! Switching to Enhanced Bug")
                self.hit_point = Point(x=self.robot_pose.x, y=self.robot_pose.y, z=0.0)
                self.current_state = NavigationState.ENHANCED_BUG
            # Stay in ENHANCED_BUG until path clears
            return
        
        # Priority 3: Motion to Goal (default)
        if self.current_state == NavigationState.ENHANCED_BUG:
            # Check if we can leave the wall
            distance_to_goal_now = self.get_distance_to_goal()
            if self.hit_point:
                dx = self.hit_point.x - self.robot_pose.x
                dy = self.hit_point.y - self.robot_pose.y
                distance_to_hit = math.sqrt(dx*dx + dy*dy)
                
                # Leave condition: closer to goal and path is clear
                if distance_to_goal_now < distance_to_hit and self.is_path_clear():
                    self.get_logger().info("Path clear! Returning to Motion-to-Goal")
                    self.current_state = NavigationState.MOTION_TO_GOAL
                    self.hit_point = None
    
    def execute_motion_to_goal(self):
        """Execute Motion-to-Goal behavior"""
        angle_to_goal = self.get_angle_to_goal()
        self.goal_angle_pub.publish(Float32(data=float(angle_to_goal)))
    
    def execute_enhanced_bug(self):
        """Execute Enhanced Bug Algorithm (wall following)"""
        if self.scan_data is None:
            return
        
        # Get wall-following direction (left side of robot)
        ranges = self.scan_data.ranges
        num_ranges = len(ranges)
        
        # Sample left side (90 degrees)
        left_index = num_ranges // 4
        left_distance = ranges[left_index] if left_index < num_ranges else float('inf')
        
        # Follow wall at desired distance
        error = left_distance - self.wall_follow_distance
        
        # Simple proportional control for wall following
        # Negative error means too close to wall -> turn right
        # Positive error means too far from wall -> turn left
        angle_correction = -np.clip(error * 30, -45.0, 45.0)  # Scale and limit
        
        # Also consider goal direction
        angle_to_goal = self.get_angle_to_goal()
        
        # Blend wall-following with goal-seeking (60% wall, 40% goal)
        blended_angle = 0.6 * angle_correction + 0.4 * angle_to_goal
        
        self.goal_angle_pub.publish(Float32(data=float(blended_angle)))
    
    def execute_tether_recovery(self):
        """Execute Tether Recovery (backtrack toward base)"""
        # Simple strategy: head back toward base station
        if self.tether_status:
            base_x = self.tether_status.base_station_pos.x
            base_y = self.tether_status.base_station_pos.y
            
            dx = base_x - self.robot_pose.x
            dy = base_y - self.robot_pose.y
            angle_to_base = math.atan2(dy, dx)
            
            angle_error = angle_to_base - self.robot_orientation
            
            # Normalize
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            self.goal_angle_pub.publish(Float32(data=float(math.degrees(angle_error))))
            
            # Check if recovered
            if not self.is_tether_critical():
                self.get_logger().info("Tether recovered! Returning to goal-seeking")
                self.current_state = NavigationState.MOTION_TO_GOAL
    
    def control_loop(self):
        """Main planning loop called by timer"""
        # State transitions
        self.state_transition()
        
        # Execute current state
        if self.current_state == NavigationState.MOTION_TO_GOAL:
            self.execute_motion_to_goal()
        elif self.current_state == NavigationState.ENHANCED_BUG:
            self.execute_enhanced_bug()
        elif self.current_state == NavigationState.TETHER_RECOVERY:
            self.execute_tether_recovery()
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HybridPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
