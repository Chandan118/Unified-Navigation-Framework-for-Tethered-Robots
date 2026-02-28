#!/usr/bin/env python3
"""
Swarm Coordinator Node (ROS 2 version)
Assigns tasks to multiple ATLAS-T robots and performs basic spacing checks.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hybrid_navigation_msgs.msg import TetherStatus
from std_msgs.msg import String

class RobotSwarmState:
    def __init__(self, name):
        self.name = name
        self.pose = None
        self.tether_status = None
        self.planner_state = "Unknown"
        self.current_task_idx = None

class SwarmCoordinatorNode(Node):
    def __init__(self):
        super().__init__('swarm_coordinator_node')

        # Parameters
        self.declare_parameter('robot_count', 10)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('goal_radius', 0.5)
        self.declare_parameter('min_spacing', 1.0)
        self.declare_parameter('task_radius', 8.0)
        self.declare_parameter('tasks_per_robot', 2)

        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value
        self.goal_radius = self.get_parameter('goal_radius').get_parameter_value().double_value
        self.min_spacing = self.get_parameter('min_spacing').get_parameter_value().double_value
        self.task_radius = self.get_parameter('task_radius').get_parameter_value().double_value
        self.tasks_per_robot = self.get_parameter('tasks_per_robot').get_parameter_value().integer_value

        # Task points and states
        self.task_points = self._generate_task_points()
        self.task_states = ["unassigned" for _ in self.task_points]

        # Per-robot state and publishers
        self.robots = {}
        self.goal_publishers = {}

        for i in range(1, self.robot_count + 1):
            robot_name = f"{self.robot_prefix}{i}"
            ns = f"/{robot_name}"

            self.robots[robot_name] = RobotSwarmState(robot_name)

            # Subscribers (dynamic names based on robot namespace)
            self.create_subscription(
                PoseWithCovarianceStamped, f"{ns}/fused_pose", 
                self._make_pose_cb(robot_name), 10
            )
            self.create_subscription(
                TetherStatus, f"{ns}/tether_status", 
                self._make_tether_cb(robot_name), 10
            )
            self.create_subscription(
                String, f"{ns}/planner_state", 
                self._make_state_cb(robot_name), 10
            )

            # Publishers
            self.goal_publishers[robot_name] = self.create_publisher(
                PoseStamped, f"{ns}/move_base_simple/goal", 10
            )

        self.timer = self.create_timer(1.0, self.update_swarm)
        self.get_logger().info(f"Swarm Coordinator initialized for {self.robot_count} robots")

    def _generate_task_points(self):
        num_tasks = max(self.robot_count * self.tasks_per_robot, 1)
        points = []
        for k in range(num_tasks):
            angle = 2.0 * math.pi * float(k) / float(num_tasks)
            x = self.task_radius * math.cos(angle)
            y = self.task_radius * math.sin(angle)
            points.append((x, y))
        return points

    def _make_pose_cb(self, robot_name):
        return lambda msg: setattr(self.robots[robot_name], 'pose', msg)

    def _make_tether_cb(self, robot_name):
        return lambda msg: setattr(self.robots[robot_name], 'tether_status', msg)

    def _make_state_cb(self, robot_name):
        return lambda msg: setattr(self.robots[robot_name], 'planner_state', msg.data)

    def _distance(self, pose_msg, x, y):
        px = pose_msg.pose.pose.position.x
        py = pose_msg.pose.pose.position.y
        return math.sqrt((px - x) ** 2 + (py - y) ** 2)

    def update_swarm(self):
        for robot_name, state in self.robots.items():
            if state.pose is None:
                continue

            # Check completion
            if state.current_task_idx is not None:
                tx, ty = self.task_points[state.current_task_idx]
                if self._distance(state.pose, tx, ty) <= self.goal_radius:
                    self.task_states[state.current_task_idx] = "completed"
                    self.get_logger().info(f"{robot_name} completed task {state.current_task_idx}")
                    state.current_task_idx = None

            # Assign new task
            if state.current_task_idx is None:
                task_idx = self._select_task_for_robot(state)
                if task_idx is not None:
                    self._send_goal(robot_name, task_idx)

        self._check_spacing()

    def _select_task_for_robot(self, state):
        best_idx, best_dist = None, None
        px = state.pose.pose.pose.position.x
        py = state.pose.pose.pose.position.y

        for idx, (tx, ty) in enumerate(self.task_points):
            if self.task_states[idx] != "unassigned":
                continue
            dist = math.sqrt((px - tx) ** 2 + (py - ty) ** 2)
            if best_idx is None or dist < best_dist:
                best_idx, best_dist = idx, dist

        if best_idx is not None:
            self.task_states[best_idx] = "in_progress"
        return best_idx

    def _send_goal(self, robot_name, task_idx):
        tx, ty = self.task_points[task_idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.orientation.w = 1.0

        if robot_name in self.goal_publishers:
            self.goal_publishers[robot_name].publish(msg)
            self.robots[robot_name].current_task_idx = task_idx
            self.get_logger().info(f"Assigned task {task_idx} to {robot_name}")

    def _check_spacing(self):
        robot_names = list(self.robots.keys())
        for i in range(len(robot_names)):
            for j in range(i + 1, len(robot_names)):
                r1, r2 = self.robots[robot_names[i]], self.robots[robot_names[j]]
                if r1.pose is None or r2.pose is None:
                    continue
                p1, p2 = r1.pose.pose.pose.position, r2.pose.pose.pose.position
                d = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
                if d < self.min_spacing:
                    self.get_logger().warn(f"Near-collision: {r1.name} and {r2.name} at {d:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
