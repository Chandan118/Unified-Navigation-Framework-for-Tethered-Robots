#!/usr/bin/env python3
"""
Mock Data Generator for ATLAS-T Simulation (ROS 2 version)
Publishes fake sensor data to allow simulation logic to run without Gazebo
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from hybrid_navigation_msgs.msg import TetherStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Quaternion
import math

class MockDataGenerator(Node):
    def __init__(self):
        super().__init__('mock_data_generator')
        self.bridge = CvBridge()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tether_pub = self.create_publisher(TetherStatus, '/tether_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.tether_len = 0.0
        
        # Timer (10Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_data)
        
        self.get_logger().info("Mock Data Generator initialized (ROS 2)")
        
    def publish_data(self):
        # Update fake state (circular motion)
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        
        self.x = 5.0 * math.cos(t * 0.1)
        self.y = 5.0 * math.sin(t * 0.1)
        self.theta = (t * 0.1) % (2 * math.pi)
        self.tether_len = math.sqrt(self.x**2 + self.y**2)
        
        stamp = now.to_msg()
        
        # Odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        q = self.get_quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)
        
        # Scan
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi/2
        scan.angle_max = math.pi/2
        scan.angle_increment = math.pi/180
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [float(2.0 + 0.1 * np.random.randn()) for _ in range(181)]
        self.scan_pub.publish(scan)
        
        # Tether
        tether = TetherStatus()
        tether.header.stamp = stamp
        tether.length = float(self.tether_len)
        tether.max_length = 30.0
        tether.tension = float(5.0 + 2.0 * np.random.randn())
        tether.snag_detected = False
        tether.base_station_pos = Point(x=0.0, y=0.0, z=0.0)
        self.tether_pub.publish(tether)
        
        # Image
        img = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        cv2.putText(img, "MOCK SIM (ROS2)", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        q.w = cr * cp * cy + sr * sp * sy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MockDataGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
