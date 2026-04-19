#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    """Publishes initial AMCL pose repeatedly during startup."""

    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("delay_sec", 5.0)
        self.declare_parameter("repeat_count", 3)
        self.declare_parameter("repeat_interval_sec", 2.0)
        self.declare_parameter("frame_id", "map")

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        delay = float(self.get_parameter("delay_sec").value)
        self.repeat_count = int(self.get_parameter("repeat_count").value)
        self.repeat_interval_sec = float(self.get_parameter("repeat_interval_sec").value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.timer = self.create_timer(delay, self._first_publish)
        self.publish_counter = 0
        self.get_logger().info(
            f"Initial pose publisher ready: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}"
        )

    def _publish_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        # Use stamp=0 so AMCL uses latest available transform.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.12
        self.pub.publish(msg)
        self.publish_counter += 1
        self.get_logger().info(f"Published /initialpose ({self.publish_counter}/{self.repeat_count}).")

    def _first_publish(self) -> None:
        self.timer.cancel()
        self._publish_pose()
        self.timer = self.create_timer(self.repeat_interval_sec, self._repeat_publish)

    def _repeat_publish(self) -> None:
        if self.publish_counter >= self.repeat_count:
            self.timer.cancel()
            return
        self._publish_pose()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
