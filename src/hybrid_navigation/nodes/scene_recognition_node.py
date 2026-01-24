#!/usr/bin/env python3
"""
Scene Recognition Node using CNN
Classifies scenes as Simple, Moderate, or Complex
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from hybrid_navigation.msg import SceneComplexity

class SceneRecognitionNode:
    def __init__(self):
        rospy.init_node('scene_recognition_node', anonymous=False)
        
        self.bridge = CvBridge()
        
        # Simple feature-based complexity calculation
        # In a real implementation, this would be a trained CNN
        self.complexity_threshold_low = 0.3
        self.complexity_threshold_high = 0.7
        
        # Publishers
        self.scene_pub = rospy.Publisher('/scene_complexity', SceneComplexity, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Scene Recognition Node initialized")
    
    def calculate_complexity(self, image):
        """
        Calculate scene complexity using image features
        In production, this would use a trained CNN model
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Calculate edge density (proxy for clutter)
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.count_nonzero(edges) / edges.size
        
        # Calculate variance (proxy for texture complexity)
        variance = np.var(gray) / 10000.0  # Normalize
        
        # Combine metrics
        complexity = (edge_density * 0.7 + variance * 0.3)
        
        return np.clip(complexity, 0, 1)
    
    def classify_scene(self, complexity):
        """Classify scene type based on complexity score"""
        if complexity < self.complexity_threshold_low:
            return "Simple"
        elif complexity < self.complexity_threshold_high:
            return "Moderate"
        else:
            return "Complex"
    
    def detect_trap(self, image):
        """
        Simple trap detection (e.g., U-shaped obstacles)
        In production, this would use trained object detection
        """
        # Placeholder: detect based on image features
        # Real implementation would use YOLO or similar
        return False
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Calculate complexity
            complexity_score = self.calculate_complexity(cv_image)
            scene_type = self.classify_scene(complexity_score)
            
            # Detect traps
            trap_detected = self.detect_trap(cv_image)
            
            # Create and publish message
            scene_msg = SceneComplexity()
            scene_msg.header.stamp = rospy.Time.now()
            scene_msg.header.frame_id = "camera_link"
            scene_msg.scene_type = scene_type
            scene_msg.complexity_score = complexity_score
            scene_msg.obstacle_density = complexity_score  # Approximation
            scene_msg.trap_detected = trap_detected
            
            self.scene_pub.publish(scene_msg)
            
            rospy.logdebug("Scene: %s, Complexity: %.3f", scene_type, complexity_score)
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    try:
        node = SceneRecognitionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
