#!/usr/bin/env python3
"""
Scene Recognition Node using CNN (ROS 2 version)
Classifies scenes as Simple, Moderate, or Complex
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from hybrid_navigation_msgs.msg import SceneComplexity

class SceneRecognitionNode(Node):
    def __init__(self):
        super().__init__('scene_recognition_node')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('complexity_threshold_low', 0.3)
        self.declare_parameter('complexity_threshold_high', 0.7)
        
        self.complexity_threshold_low = self.get_parameter('complexity_threshold_low').get_parameter_value().double_value
        self.complexity_threshold_high = self.get_parameter('complexity_threshold_high').get_parameter_value().double_value
        
        # Publishers
        self.scene_pub = self.create_publisher(SceneComplexity, '/scene_complexity', 10)
        
        # Subscribers
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        
        self.get_logger().info("Scene Recognition Node initialized (ROS 2)")
    
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
        
        return np.clip(complexity, 0.0, 1.0)
    
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
        """
        # Placeholder
        return False
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Calculate complexity
            complexity_score = float(self.calculate_complexity(cv_image))
            scene_type = self.classify_scene(complexity_score)
            
            # Detect traps
            trap_detected = self.detect_trap(cv_image)
            
            # Create and publish message
            scene_msg = SceneComplexity()
            scene_msg.header.stamp = self.get_clock().now().to_msg()
            scene_msg.header.frame_id = "camera_link"
            scene_msg.scene_type = scene_type
            scene_msg.complexity_score = complexity_score
            scene_msg.obstacle_density = complexity_score  # Approximation
            scene_msg.trap_detected = trap_detected
            
            self.scene_pub.publish(scene_msg)
            
            self.get_logger().debug(f"Scene: {scene_type}, Complexity: {complexity_score:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SceneRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
