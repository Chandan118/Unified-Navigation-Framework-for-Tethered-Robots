#!/usr/bin/env python3
"""
Swarm Data Logger Node for ATLAS-T Simulation (ROS 2 version)
Records per-robot metrics to a CSV file for swarm performance analysis.
"""

import csv
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from hybrid_navigation_msgs.msg import TetherStatus, SceneComplexity
from std_msgs.msg import String

class SwarmDataLoggerNode(Node):
    def __init__(self):
        super().__init__('swarm_data_logger_node')

        # Parameters
        default_dir = os.path.join(os.environ.get("HOME", "/tmp"), "atlas_ws", "results")
        self.declare_parameter('log_dir', default_dir)
        self.declare_parameter('robot_count', 10)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('log_rate', 5.0)

        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value
        self.log_rate = self.get_parameter('log_rate').get_parameter_value().double_value

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        self.filename = os.path.join(
            self.log_dir, f"sim_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )

        # Per-robot data storage
        self.robot_data = {}
        for i in range(1, self.robot_count + 1):
            robot_id = f"{self.robot_prefix}{i}"
            self.robot_data[robot_id] = {
                "timestamp": 0.0,
                "robot_id": robot_id,
                "pos_x": 0.0,
                "pos_y": 0.0,
                "tether_length": 0.0,
                "tether_tension": 0.0,
                "complexity_score": 0.0,
                "planner_state": "Unknown",
            }

        # CSV setup
        self.fieldnames = [
            "timestamp", "robot_id", "pos_x", "pos_y", 
            "tether_length", "tether_tension", "complexity_score", "planner_state"
        ]
        self.csv_file = open(self.filename, "w", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()

        # Subscribers per robot
        for i in range(1, self.robot_count + 1):
            robot_id = f"{self.robot_prefix}{i}"
            ns = f"/{robot_id}"

            self.create_subscription(Odometry, f"{ns}/odom", self._make_odom_cb(robot_id), 10)
            self.create_subscription(TetherStatus, f"{ns}/tether_status", self._make_tether_cb(robot_id), 10)
            self.create_subscription(SceneComplexity, f"{ns}/scene_complexity", self._make_scene_cb(robot_id), 10)
            self.create_subscription(String, f"{ns}/planner_state", self._make_state_cb(robot_id), 10)

        # Timer for periodic logging
        period = 1.0 / self.log_rate if self.log_rate > 0 else 0.2
        self.timer = self.create_timer(period, self.log_data)

        self.get_logger().info(f"Swarm Data Logger initialized. Saving to {self.filename}")

    def _make_odom_cb(self, robot_id):
        return lambda msg: self._update_data(robot_id, {'pos_x': msg.pose.pose.position.x, 'pos_y': msg.pose.pose.position.y})

    def _make_tether_cb(self, robot_id):
        return lambda msg: self._update_data(robot_id, {'tether_length': msg.length, 'tether_tension': msg.tension})

    def _make_scene_cb(self, robot_id):
        return lambda msg: self._update_data(robot_id, {'complexity_score': msg.complexity_score})

    def _make_state_cb(self, robot_id):
        return lambda msg: self._update_data(robot_id, {'planner_state': msg.data})

    def _update_data(self, robot_id, new_values):
        if robot_id in self.robot_data:
            self.robot_data[robot_id].update(new_values)

    def log_data(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for robot_id, data in self.robot_data.items():
            data["timestamp"] = now
            self.csv_writer.writerow(data)
        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, "csv_file"):
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SwarmDataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
