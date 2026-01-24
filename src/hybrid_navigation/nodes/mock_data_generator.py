#!/usr/bin/env python3
"""
Mock Data Generator for ATLAS-T Simulation
Publishes fake sensor data to allow simulation logic to run without Gazebo
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from hybrid_navigation.msg import TetherStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion

class MockDataGenerator:
    def __init__(self):
        rospy.init_node('mock_data_generator', anonymous=False)
        self.bridge = CvBridge()
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=1)
        self.tether_pub = rospy.Publisher('/tether_status', TetherStatus, queue_size=1)
        self.image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.tether_len = 0.0
        
        # Timer (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.publish_data)
        
        rospy.loginfo("Mock Data Generator initialized")
        
    def publish_data(self, event):
        # Update fake state (circular motion)
        t = rospy.get_time()
        self.x = 5.0 * np.cos(t * 0.1)
        self.y = 5.0 * np.sin(t * 0.1)
        self.theta = (t * 0.1) % (2 * np.pi)
        self.tether_len = np.sqrt(self.x**2 + self.y**2)
        
        # Odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position = Point(self.x, self.y, 0)
        q = self.get_quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)
        
        # Scan
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = -np.pi/2
        scan.angle_max = np.pi/2
        scan.angle_increment = np.pi/180
        scan.range_min = 0.1
        scan.range_max = 30.0
        # Add a "wall" at 2 meters
        scan.ranges = [2.0 + 0.1 * np.random.randn() for _ in range(181)]
        self.scan_pub.publish(scan)
        
        # Tether
        tether = TetherStatus()
        tether.header.stamp = rospy.Time.now()
        tether.length = self.tether_len
        tether.max_length = 30.0
        tether.tension = 5.0 + 2.0 * np.random.randn()
        tether.snag_detected = False
        tether.base_station_pos = Point(0, 0, 0)
        self.tether_pub.publish(tether)
        
        # Image (just a random noise image)
        img = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        # Draw some "complexity"
        cv2.putText(img, "MOCK SIM", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

if __name__ == '__main__':
    try:
        gen = MockDataGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
