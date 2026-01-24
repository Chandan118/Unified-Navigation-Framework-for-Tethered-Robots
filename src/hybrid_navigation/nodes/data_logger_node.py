#!/usr/bin/env python3
"""
Data Logger Node for ATLAS-T Simulation
Records simulation metrics to a CSV file for performance analysis
"""

import rospy
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from hybrid_navigation.msg import TetherStatus, SceneComplexity
from std_msgs.msg import String

class DataLoggerNode:
    def __init__(self):
        rospy.init_node('data_logger_node', anonymous=False)
        
        # Parameters
        self.log_dir = rospy.get_param('~log_dir', os.path.join(os.environ['HOME'], 'atlas_ws/results'))
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        self.filename = os.path.join(self.log_dir, f"sim_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        
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
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/tether_status', TetherStatus, self.tether_callback)
        rospy.Subscriber('/scene_complexity', SceneComplexity, self.scene_callback)
        rospy.Subscriber('/planner_state', String, self.state_callback)
        
        # Timer for logging (5Hz)
        rospy.Timer(rospy.Duration(0.2), self.log_data)
        
        rospy.loginfo(f"Data Logger initialized. Saving to {self.filename}")
        
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
        
    def log_data(self, event):
        if rospy.is_shutdown():
            return
            
        self.current_data['timestamp'] = rospy.get_time()
        self.csv_writer.writerow(self.current_data)
        self.csv_file.flush()
        
    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()

if __name__ == '__main__':
    try:
        node = DataLoggerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
