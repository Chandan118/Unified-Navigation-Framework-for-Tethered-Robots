#!/usr/bin/env python3
"""
Sensor Fusion Node with Extended Kalman Filter (ROS 2 version)
Implements dynamic weight adjustment based on sensor variance (Eq. 8-9 from paper)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # State: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Covariance matrix
        
        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise (will be adapted dynamically)
        self.R_odom = np.eye(3) * 0.1
        self.R_imu = np.eye(3) * 0.05
        
        # Dynamic weights (alpha_1, alpha_2 from paper)
        self.weight_odom = 0.5
        self.weight_imu = 0.5
        
        # Sensor data buffers
        self.odom_data = None
        self.imu_data = None
        self.last_update_time = self.get_clock().now()
        
        # Variance trackers for dynamic weight adjustment
        self.odom_variance_history = []
        self.imu_variance_history = []
        self.variance_window_size = 10
        
        # Publishers
        self.fused_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fused_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Timer for EKF update (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.ekf_update)
        
        self.get_logger().info("Sensor Fusion Node initialized (ROS 2)")
    
    def odom_callback(self, msg):
        """Store odometry data"""
        self.odom_data = msg
        
        # Track variance
        # msg.pose.covariance is a list of 36 floats
        variance = np.mean(msg.pose.covariance[0:3])  # Average of x, y, z variance
        self.odom_variance_history.append(variance)
        if len(self.odom_variance_history) > self.variance_window_size:
            self.odom_variance_history.pop(0)
    
    def imu_callback(self, msg):
        """Store IMU data"""
        self.imu_data = msg
        
        # Track variance
        variance = np.mean(msg.orientation_covariance[0:3])
        self.imu_variance_history.append(variance)
        if len(self.imu_variance_history) > self.variance_window_size:
            self.imu_variance_history.pop(0)
    
    def calculate_dynamic_weights(self):
        """
        Calculate dynamic weights based on sensor variance (Eq. 8-9 from paper)
        Lower variance = higher weight
        """
        if len(self.odom_variance_history) > 3 and len(self.imu_variance_history) > 3:
            avg_odom_var = np.mean(self.odom_variance_history)
            avg_imu_var = np.mean(self.imu_variance_history)
            
            if avg_odom_var + avg_imu_var > 0:
                self.weight_odom = avg_imu_var / (avg_odom_var + avg_imu_var)
                self.weight_imu = avg_odom_var / (avg_odom_var + avg_imu_var)
            else:
                self.weight_odom = 0.5
                self.weight_imu = 0.5
    
    def predict(self, dt):
        """EKF Prediction step"""
        # State transition model (simple constant velocity)
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        # Predict state
        self.state = F @ self.state
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update_with_odom(self):
        """EKF Update step with odometry"""
        if self.odom_data is None:
            return
        
        # Measurement model (observe x, y, theta)
        H = np.zeros((3, 6))
        H[0, 0] = 1; H[1, 1] = 1; H[2, 2] = 1
        
        x_meas = self.odom_data.pose.pose.position.x
        y_meas = self.odom_data.pose.pose.position.y
        
        # Quaternion to theta
        q = self.odom_data.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta_meas = math.atan2(siny_cosp, cosy_cosp)
        
        z = np.array([x_meas, y_meas, theta_meas])
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R_odom * self.weight_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def update_with_imu(self):
        """EKF Update step with IMU"""
        if self.imu_data is None:
            return
        
        # Measurement model (observe angular velocity)
        H = np.zeros((1, 6))
        H[0, 5] = 1
        
        omega_meas = self.imu_data.angular_velocity.z
        z = np.array([omega_meas])
        y = z - H @ self.state
        S = H @ self.P @ H.T + np.array([[self.R_imu[2, 2] * self.weight_imu]])
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def ekf_update(self):
        """Main EKF update loop"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        
        self.calculate_dynamic_weights()
        
        if 0 < dt < 1.0:
            self.predict(dt)
        
        self.update_with_odom()
        self.update_with_imu()
        
        self.publish_fused_pose()
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Simple euler to quaternion conversion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy # x
        q[1] = cr * sp * cy + sr * cp * sy # y
        q[2] = cr * cp * sy - sr * sp * cy # z
        q[3] = cr * cp * cy + sr * sp * sy # w
        return q
    
    def publish_fused_pose(self):
        """Publish fused pose and broadcast TF"""
        now = self.get_clock().now().to_msg()
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Covariance
        cov = np.zeros(36)
        cov[0] = self.P[0, 0]
        cov[7] = self.P[1, 1]
        cov[35] = self.P[2, 2]
        msg.pose.covariance = cov.tolist()
        
        self.fused_pose_pub.publish(msg)
        
        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
