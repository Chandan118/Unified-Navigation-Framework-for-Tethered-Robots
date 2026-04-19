#!/usr/bin/env python3
import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, Float32


class SafetyCmdVelFilter(Node):
    """Filters navigation velocity commands with hard safety constraints."""

    def __init__(self) -> None:
        super().__init__("safety_cmd_vel_filter")

        self.declare_parameter("input_cmd_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("ultrasonic_topic", "/sensor/ultrasonic")
        self.declare_parameter("camera_obstacle_topic", "/camera/obstacle")
        self.declare_parameter("camera_score_topic", "/camera/obstacle_score")
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("stop_distance_m", 0.28)
        self.declare_parameter("slow_distance_m", 0.45)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.9)
        self.declare_parameter("max_linear_accel", 0.4)
        self.declare_parameter("max_angular_accel", 1.6)
        self.declare_parameter("camera_slowdown_factor", 0.5)
        self.declare_parameter("camera_trigger_score", 0.22)

        self.input_cmd_topic = self.get_parameter("input_cmd_topic").value
        self.output_cmd_topic = self.get_parameter("output_cmd_topic").value
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.stop_distance_m = float(self.get_parameter("stop_distance_m").value)
        self.slow_distance_m = float(self.get_parameter("slow_distance_m").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)
        self.camera_slowdown_factor = float(self.get_parameter("camera_slowdown_factor").value)
        self.camera_trigger_score = float(self.get_parameter("camera_trigger_score").value)

        self.latest_nav_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.min_front_scan = float("inf")
        self.ultrasonic_range_m = float("inf")
        self.camera_obstacle = False
        self.camera_score = 0.0
        self.last_published = Twist()

        self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 20)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 20
        )
        self.create_subscription(
            Range, self.get_parameter("ultrasonic_topic").value, self.ultrasonic_callback, 20
        )
        self.create_subscription(
            Bool, self.get_parameter("camera_obstacle_topic").value, self.camera_obs_callback, 20
        )
        self.create_subscription(
            Float32, self.get_parameter("camera_score_topic").value, self.camera_score_callback, 20
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 20)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"Safety filter ready: input={self.input_cmd_topic}, output={self.output_cmd_topic}"
        )

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_nav_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        distances = self._front_sector(msg.ranges)
        if distances:
            self.min_front_scan = min(distances)
        else:
            self.min_front_scan = float("inf")

    def ultrasonic_callback(self, msg: Range) -> None:
        if math.isfinite(msg.range) and msg.range > 0.0:
            self.ultrasonic_range_m = msg.range

    def camera_obs_callback(self, msg: Bool) -> None:
        self.camera_obstacle = bool(msg.data)

    def camera_score_callback(self, msg: Float32) -> None:
        self.camera_score = float(msg.data)

    def control_loop(self) -> None:
        now = self.get_clock().now()
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9

        target = Twist()
        if cmd_age <= self.command_timeout_sec:
            target = self.latest_nav_cmd

        target.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, target.linear.x))
        target.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, target.angular.z))

        nearest = min(self.min_front_scan, self.ultrasonic_range_m)
        hard_stop = nearest < self.stop_distance_m
        should_slow = nearest < self.slow_distance_m
        camera_slow = self.camera_obstacle and self.camera_score >= self.camera_trigger_score

        if hard_stop and target.linear.x > 0.0:
            target.linear.x = 0.0
            # Rotate in place to help local planner recover.
            if abs(target.angular.z) < 0.35:
                target.angular.z = 0.45
        elif should_slow and target.linear.x > 0.0:
            target.linear.x *= 0.35

        if camera_slow and target.linear.x > 0.0:
            target.linear.x *= self.camera_slowdown_factor

        limited = self._limit_accel(self.last_published, target, dt=0.05)
        self.last_published = limited
        self.cmd_pub.publish(limited)

    def _front_sector(self, ranges: List[float]) -> List[float]:
        if not ranges:
            return []
        n = len(ranges)
        # Assume center index points forward for a merged scan.
        start = int(n * 0.42)
        end = int(n * 0.58)
        valid = []
        for r in ranges[start:end]:
            if math.isfinite(r) and r > 0.05:
                valid.append(r)
        return valid

    def _limit_accel(self, prev: Twist, target: Twist, dt: float) -> Twist:
        out = Twist()
        max_dv = self.max_linear_accel * dt
        max_dw = self.max_angular_accel * dt
        out.linear.x = prev.linear.x + self._clamp(target.linear.x - prev.linear.x, -max_dv, max_dv)
        out.angular.z = prev.angular.z + self._clamp(
            target.angular.z - prev.angular.z, -max_dw, max_dw
        )
        return out

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyCmdVelFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
