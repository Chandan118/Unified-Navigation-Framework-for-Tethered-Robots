#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class SmallRobotNavigator(Node):
    """Simple navigation node for a small robot.

    - Subscribes to odom, lidar, imu, ultrasonic, and camera.
    - Publishes /cmd_vel for basic go-straight-and-avoid behavior.
    """

    def __init__(self) -> None:
        super().__init__('small_robot_navigator')

        # State
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0
        self.lidar_ranges = []
        self.ultrasonic_distances = [0.0, 0.0, 0.0, 0.0]

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.ultrasonic_sub = self.create_subscription(Float32MultiArray, '/ultrasonic_distances', self.ultrasonic_callback, 10)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)

        # Parameters
        self.max_linear_speed = self.declare_parameter('max_linear_speed', 0.3).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 1.0).value

        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Small Robot Navigator (tethered_navigation) initialized')

    # Callbacks
    def odom_callback(self, msg: Odometry) -> None:
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

    def laser_callback(self, msg: LaserScan) -> None:
        self.lidar_ranges = list(msg.ranges)

    def imu_callback(self, msg: Imu) -> None:
        # Simple yaw extraction from quaternion (approx)
        z = msg.orientation.z
        w = msg.orientation.w
        self.heading = 2.0 * math.atan2(z, w)

    def ultrasonic_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            self.ultrasonic_distances = list(msg.data[:4])

    def camera_callback(self, msg: Image) -> None:
        # For now we just confirm images are received
        pass

    # Control
    def control_loop(self) -> None:
        cmd = Twist()

        # Basic obstacle avoidance using lidar front sector
        obstacle_close = False
        if self.lidar_ranges:
            n = len(self.lidar_ranges)
            start = int(0.45 * n)
            end = int(0.55 * n)
            front = [r for r in self.lidar_ranges[start:end] if r > 0.2]
            if front and min(front) < 0.4:
                obstacle_close = True

        if obstacle_close:
            # Stop and rotate a bit
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed * 0.5
            self.get_logger().debug('Obstacle detected, rotating in place')
        else:
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmallRobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Small Robot Navigator shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
