#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from hybrid_navigation.msg import TetherStatus

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.tether_sub = self.create_subscription(TetherStatus, '/tether_status', self.tether_callback, 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
        
        self.get_logger().info('ATLAS-T Mission Manager Initialized')
        self.current_tether_tension = 0.0
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def tether_callback(self, msg):
        self.current_tether_tension = msg.tension
        if self.current_tether_tension > 20.0:
            self.get_logger().warn('High tether tension detected! Modifying mission strategy.')
            msg_out = String()
            msg_out.data = 'EMERGENCY_RECOVERY'
            self.mission_status_pub.publish(msg_out)

    def timer_callback(self):
        msg = String()
        msg.data = 'MISSION_ACTIVE'
        self.mission_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    manager = MissionManager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
