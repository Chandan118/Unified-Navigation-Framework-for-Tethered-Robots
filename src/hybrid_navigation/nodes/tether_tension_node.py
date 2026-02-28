#!/usr/bin/env python3
"""
Tether Tension and Snag Detection Node (ROS 2 version)
Calculates virtual tether tension and detects snags based on geometry
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from hybrid_navigation_msgs.msg import TetherStatus
from visualization_msgs.msg import Marker
import math

class TetherTensionNode(Node):
    def __init__(self):
        super().__init__('tether_tension_node')
        
        # Parameters
        self.declare_parameter('max_tether_length', 30.0)
        self.declare_parameter('tension_coefficient', 1.5)
        
        self.max_tether_length = self.get_parameter('max_tether_length').get_parameter_value().double_value
        self.tension_coefficient = self.get_parameter('tension_coefficient').get_parameter_value().double_value
        
        self.base_station_pos = Point(x=0.0, y=0.0, z=0.0)
        self.robot_pos = Point(x=0.0, y=0.0, z=0.0)
        
        # Path breadcrumbs for snag detection
        self.breadcrumbs = []
        self.breadcrumb_distance = 0.5  # meters between breadcrumbs
        
        # Costmap for snag detection
        self.costmap = None
        self.costmap_info = None
        
        # Publishers
        self.tether_status_pub = self.create_publisher(TetherStatus, '/tether_status', 10)
        self.tether_viz_pub = self.create_publisher(Marker, '/tether_visualization', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/move_base/local_costmap/costmap', self.costmap_callback, 10)
        
        # Timer for publishing (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_tether_status)
        
        self.get_logger().info("Tether Tension Node initialized (ROS 2)")
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_pos.x = msg.pose.pose.position.x
        self.robot_pos.y = msg.pose.pose.position.y
        self.robot_pos.z = msg.pose.pose.position.z
        
        # Add breadcrumb if robot has moved enough
        if self.should_add_breadcrumb():
            self.breadcrumbs.append(Point(x=self.robot_pos.x, y=self.robot_pos.y, z=0.0))
            
            # Limit breadcrumb list size
            if len(self.breadcrumbs) > 100:
                self.breadcrumbs.pop(0)
    
    def costmap_callback(self, msg):
        """Store costmap for snag detection"""
        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.costmap_info = msg.info
    
    def should_add_breadcrumb(self):
        """Check if we should add a new breadcrumb"""
        if not self.breadcrumbs:
            return True
        
        last_crumb = self.breadcrumbs[-1]
        distance = math.sqrt(
            (self.robot_pos.x - last_crumb.x)**2 + 
            (self.robot_pos.y - last_crumb.y)**2
        )
        
        return distance >= self.breadcrumb_distance
    
    def calculate_tether_length(self):
        """Calculate current tether length (Euclidean distance)"""
        dx = self.robot_pos.x - self.base_station_pos.x
        dy = self.robot_pos.y - self.base_station_pos.y
        dz = self.robot_pos.z - self.base_station_pos.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_tension(self, length):
        """Calculate virtual tension based on tether length"""
        if length < self.max_tether_length * 0.7:
            # Low tension zone
            tension = length * 0.5
        elif length < self.max_tether_length * 0.9:
            # Medium tension zone
            tension = 10 + (length - self.max_tether_length * 0.7) * 1.0
        else:
            # High tension zone
            tension = 20 + (length - self.max_tether_length * 0.9) * 3.0
        
        # Cap at 50N
        return min(tension, 50.0)
    
    def detect_snag(self):
        """Detect if tether is snagged on an obstacle"""
        if self.costmap is None or len(self.breadcrumbs) < 2:
            return False
        
        # Check line segments between breadcrumbs
        for i in range(len(self.breadcrumbs) - 1):
            p1 = self.breadcrumbs[i]
            p2 = self.breadcrumbs[i + 1]
            if self.line_intersects_obstacle(p1, p2):
                return True
        
        # Check from last breadcrumb to base station
        if self.breadcrumbs:
            base_p = Point(x=self.base_station_pos.x, y=self.base_station_pos.y, z=0.0)
            if self.line_intersects_obstacle(self.breadcrumbs[-1], base_p):
                return True
        
        return False
    
    def line_intersects_obstacle(self, p1, p2):
        """Check if line segment intersects with obstacles in costmap"""
        if self.costmap_info is None:
            return False
        
        # Sample points along the line
        num_samples = 20
        for i in range(num_samples):
            t = i / float(num_samples)
            x = p1.x * (1 - t) + p2.x * t
            y = p1.y * (1 - t) + p2.y * t
            
            # Convert to costmap coordinates
            map_x = int((x - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
            map_y = int((y - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
            
            # Check bounds
            if 0 <= map_x < self.costmap_info.width and 0 <= map_y < self.costmap_info.height:
                # OccupancyGrid values are [0, 100], -1 is unknown. 
                # > 50 is typically obstacle.
                if self.costmap[map_y, map_x] > 50:
                    return True
        
        return False
    
    def publish_tether_status(self):
        """Publish tether status message"""
        # Calculate values
        length = self.calculate_tether_length()
        tension = self.calculate_tension(length)
        snag_detected = self.detect_snag()
        
        # Create message
        msg = TetherStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.length = length
        msg.tension = tension
        msg.max_length = self.max_tether_length
        msg.snag_detected = snag_detected
        msg.base_station_pos = self.base_station_pos
        msg.robot_pos = self.robot_pos
        
        self.tether_status_pub.publish(msg)
        
        # Visualize
        self.publish_visualization(length, tension, snag_detected)
    
    def publish_visualization(self, length, tension, snag_detected):
        """Publish tether visualization marker"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tether"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Add points
        marker.points.append(self.base_station_pos)
        for crumb in self.breadcrumbs:
            marker.points.append(crumb)
        marker.points.append(self.robot_pos)
        
        # Color
        if snag_detected:
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        elif tension > 30.0:
            marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0
        elif tension > 15.0:
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0
        else:
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
        
        marker.color.a = 0.8
        marker.scale.x = 0.02
        
        self.tether_viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TetherTensionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
