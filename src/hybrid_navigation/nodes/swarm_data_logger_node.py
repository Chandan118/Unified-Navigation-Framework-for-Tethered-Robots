#!/usr/bin/env python3
"""
Swarm Data Logger Node for ATLAS-T Simulation
Records per-robot metrics to a CSV file for swarm performance analysis.
"""

import csv
import os
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry
from hybrid_navigation.msg import TetherStatus, SceneComplexity
from std_msgs.msg import String


class SwarmDataLoggerNode:
    def __init__(self):
        rospy.init_node("swarm_data_logger_node", anonymous=False)

        # Parameters
        default_dir = os.path.join(os.environ.get("HOME", "/tmp"), "atlas_ws", "results")
        self.log_dir = rospy.get_param("~log_dir", default_dir)
        self.robot_count = rospy.get_param("~robot_count", 10)
        self.robot_prefix = rospy.get_param("~robot_prefix", "robot_")
        self.log_rate = rospy.get_param("~log_rate", 5.0)  # Hz

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
            "timestamp",
            "robot_id",
            "pos_x",
            "pos_y",
            "tether_length",
            "tether_tension",
            "complexity_score",
            "planner_state",
        ]
        self.csv_file = open(self.filename, "w", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()

        # Subscribers per robot
        for i in range(1, self.robot_count + 1):
            robot_id = f"{self.robot_prefix}{i}"
            ns = f"/{robot_id}"

            rospy.Subscriber(f"{ns}/odom", Odometry, self._make_odom_cb(robot_id))
            rospy.Subscriber(f"{ns}/tether_status", TetherStatus, self._make_tether_cb(robot_id))
            rospy.Subscriber(
                f"{ns}/scene_complexity", SceneComplexity, self._make_scene_cb(robot_id)
            )
            rospy.Subscriber(f"{ns}/planner_state", String, self._make_state_cb(robot_id))

        # Timer for periodic logging
        period = 1.0 / self.log_rate if self.log_rate > 0 else 0.2
        rospy.Timer(rospy.Duration(period), self.log_data)

        rospy.loginfo("Swarm Data Logger initialized. Saving to %s", self.filename)

    def _make_odom_cb(self, robot_id):
        def cb(msg):
            data = self.robot_data[robot_id]
            data["pos_x"] = msg.pose.pose.position.x
            data["pos_y"] = msg.pose.pose.position.y

        return cb

    def _make_tether_cb(self, robot_id):
        def cb(msg):
            data = self.robot_data[robot_id]
            data["tether_length"] = msg.length
            data["tether_tension"] = msg.tension

        return cb

    def _make_scene_cb(self, robot_id):
        def cb(msg):
            data = self.robot_data[robot_id]
            data["complexity_score"] = msg.complexity_score

        return cb

    def _make_state_cb(self, robot_id):
        def cb(msg):
            data = self.robot_data[robot_id]
            data["planner_state"] = msg.data

        return cb

    def log_data(self, event):
        if rospy.is_shutdown():
            return

        now = rospy.get_time()
        for robot_id, data in self.robot_data.items():
            data["timestamp"] = now
            self.csv_writer.writerow(data)
        self.csv_file.flush()

    def __del__(self):
        if hasattr(self, "csv_file"):
            self.csv_file.close()


if __name__ == "__main__":
    try:
        node = SwarmDataLoggerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
