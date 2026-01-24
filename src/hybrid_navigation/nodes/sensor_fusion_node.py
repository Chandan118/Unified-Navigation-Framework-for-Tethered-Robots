#!/usr/bin/env python3
"""
Sensor Fusion Node with Extended Kalman Filter
Implements dynamic weight adjustment based on sensor variance (Eq. 8-9 from paper)
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion_node', anonymous=False)
        
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
        self.last_update_time = rospy.Time.now()
        
        # Variance trackers for dynamic weight adjustment
        self.odom_variance_history = []
        self.imu_variance_history = []
        self.variance_window_size = 10
        
        # Publishers
        self.fused_pose_pub = rospy.Publisher('/fused_pose', PoseWithCovarianceStamped, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Timer for EKF update
        rospy.Timer(rospy.Duration(0.1), self.ekf_update)
        
        rospy.loginfo("Sensor Fusion Node initialized")
    
    def odom_callback(self, msg):
        """Store odometry data"""
        self.odom_data = msg
        
        # Track variance
        if hasattr(msg.pose, 'covariance'):
            variance = np.mean(msg.pose.covariance[0:3])  # x, y variance
            self.odom_variance_history.append(variance)
            if len(self.odom_variance_history) > self.variance_window_size:
                self.odom_variance_history.pop(0)
    
    def imu_callback(self, msg):
        """Store IMU data"""
        self.imu_data = msg
        
        # Track variance
        if hasattr(msg.orientation_covariance, '__iter__'):
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
            
            # Avoid division by zero
            if avg_odom_var + avg_imu_var > 0:
                # Inverse variance weighting
                self.weight_odom = avg_imu_var / (avg_odom_var + avg_imu_var)
                self.weight_imu = avg_odom_var / (avg_odom_var + avg_imu_var)
            else:
                self.weight_odom = 0.5
                self.weight_imu = 0.5
            
            rospy.logdebug("Dynamic weights: odom=%.3f, imu=%.3f", self.weight_odom, self.weight_imu)
    
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
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # theta
        
        # Get measurement
        x_meas = self.odom_data.pose.pose.position.x
        y_meas = self.odom_data.pose.pose.position.y
        
        # Extract theta from quaternion
        orientation_q = self.odom_data.pose.pose.orientation
        _, _, theta_meas = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        z = np.array([x_meas, y_meas, theta_meas])
        
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_odom * self.weight_odom
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def update_with_imu(self):
        """EKF Update step with IMU"""
        if self.imu_data is None:
            return
        
        # Measurement model (observe angular velocity)
        H = np.zeros((1, 6))
        H[0, 5] = 1  # omega
        
        # Get measurement
        omega_meas = self.imu_data.angular_velocity.z
        z = np.array([omega_meas])
        
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + np.array([[self.R_imu[2, 2] * self.weight_imu]])
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def ekf_update(self, event):
        """Main EKF loop"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time
        
        # Calculate dynamic weights
        self.calculate_dynamic_weights()
        
        # Prediction
        if dt > 0 and dt < 1.0:  # Sanity check
            self.predict(dt)
        
        # Update with sensors
        self.update_with_odom()
        self.update_with_imu()
        
        # Publish fused pose
        self.publish_fused_pose()
    
    def publish_fused_pose(self):
        """Publish fused pose estimate"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        
        # Copy covariance
        covariance = np.zeros(36)
        covariance[0] = self.P[0, 0]    # x
        covariance[7] = self.P[1, 1]    # y
        covariance[35] = self.P[2, 2]   # theta
        msg.pose.covariance = covariance.tolist()
        
        self.fused_pose_pub.publish(msg)
        
        # Broadcast TF
        self.tf_broadcaster.sendTransform(
            (self.state[0], self.state[1], 0),
            quaternion,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        node = SensorFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
