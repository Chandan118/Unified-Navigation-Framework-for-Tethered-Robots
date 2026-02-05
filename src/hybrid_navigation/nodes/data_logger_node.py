#!/usr/bin/env python3
"""
Data Logger Node for ATLAS-T Simulation (ROS 2 version)
Records simulation metrics to a CSV file for performance analysis
"""

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from hybrid_navigation_msgs.msg import TetherStatus, SceneComplexity
from std_msgs.msg import String

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        
        # Parameters
        self.declare_parameter('log_dir', os.path.join(os.environ['HOME'], 'atlas_ws/results'))
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        self.filename = os.path.join(log_dir, f"sim_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        
        # Data storage
        self.current_data = {
            'timestamp': 0.0,
            'pos_x': 0.0,
            'pos_y': 0.0,
            'tether_length': 0.0,
            'tether_tension': 0.0,
            'complexity_score': 0.0,
            'scene_type': 'Unknown',
            'planner_state': 'Unknown'
        }
        
        # CSV Setup
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.current_data.keys())
        self.csv_writer.writeheader()
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(TetherStatus, '/tether_status', self.tether_callback, 10)
        self.create_subscription(SceneComplexity, '/scene_complexity', self.scene_callback, 10)
        self.create_subscription(String, '/planner_state', self.state_callback, 10)
        
        # Timer for logging (5Hz)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.log_data)
        
        self.get_logger().info(f"Data Logger initialized. Saving to {self.filename}")
        
    def odom_callback(self, msg):
        self.current_data['pos_x'] = msg.pose.pose.position.x
        self.current_data['pos_y'] = msg.pose.pose.position.y
        
    def tether_callback(self, msg):
        self.current_data['tether_length'] = msg.length
        self.current_data['tether_tension'] = msg.tension
        
    def scene_callback(self, msg):
        self.current_data['complexity_score'] = msg.complexity_score
        self.current_data['scene_type'] = msg.scene_type
        
    def state_callback(self, msg):
        self.current_data['planner_state'] = msg.data
        
    def log_data(self):
        # Update timestamp with current system time in seconds
        now = self.get_clock().now()
        self.current_data['timestamp'] = now.nanoseconds / 1e9
        
        self.csv_writer.writerow(self.current_data)
        self.csv_file.flush()
        
    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
