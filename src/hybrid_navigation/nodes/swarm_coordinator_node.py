#!/usr/bin/env python3
"""
Swarm Coordinator Node
Assigns tasks to multiple ATLAS-T robots and performs basic spacing checks.
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hybrid_navigation.msg import TetherStatus
from std_msgs.msg import String


class RobotSwarmState:
    def __init__(self, name):
        self.name = name
        self.pose = None  # type: PoseWithCovarianceStamped
        self.tether_status = None  # type: TetherStatus
        self.planner_state = "Unknown"
        self.current_task_idx = None


class SwarmCoordinatorNode:
    def __init__(self):
        rospy.init_node("swarm_coordinator_node", anonymous=False)

        self.robot_count = rospy.get_param("~robot_count", 10)
        self.robot_prefix = rospy.get_param("~robot_prefix", "robot_")
        self.goal_radius = rospy.get_param("~goal_radius", 0.5)
        self.min_spacing = rospy.get_param("~min_spacing", 1.0)
        self.task_radius = rospy.get_param("~task_radius", 8.0)
        self.tasks_per_robot = rospy.get_param("~tasks_per_robot", 2)

        # Generate a simple ring of task points around the origin
        self.task_points = self._generate_task_points()
        # Task state: "unassigned", "in_progress", "completed"
        self.task_states = ["unassigned" for _ in self.task_points]

        # Per-robot state and publishers
        self.robots = {}
        self.goal_publishers = {}

        for i in range(1, self.robot_count + 1):
            robot_name = f"{self.robot_prefix}{i}"
            ns = f"/{robot_name}"

            state = RobotSwarmState(robot_name)
            self.robots[robot_name] = state

            # Subscribers for each robot
            rospy.Subscriber(f"{ns}/fused_pose", PoseWithCovarianceStamped, self._make_pose_cb(robot_name))
            rospy.Subscriber(f"{ns}/tether_status", TetherStatus, self._make_tether_cb(robot_name))
            rospy.Subscriber(f"{ns}/planner_state", String, self._make_state_cb(robot_name))

            # Publisher for robot goals
            self.goal_publishers[robot_name] = rospy.Publisher(
                f"{ns}/move_base_simple/goal", PoseStamped, queue_size=1
            )

        self.update_timer = rospy.Timer(rospy.Duration(1.0), self.update_swarm)
        rospy.loginfo(
            "Swarm Coordinator initialized for %d robots with prefix '%s'",
            self.robot_count,
            self.robot_prefix,
        )

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
        def cb(msg):
            self.robots[robot_name].pose = msg

        return cb

    def _make_tether_cb(self, robot_name):
        def cb(msg):
            self.robots[robot_name].tether_status = msg

        return cb

    def _make_state_cb(self, robot_name):
        def cb(msg):
            self.robots[robot_name].planner_state = msg.data

        return cb

    def _distance(self, pose_msg, x, y):
        px = pose_msg.pose.pose.position.x
        py = pose_msg.pose.pose.position.y
        return math.sqrt((px - x) ** 2 + (py - y) ** 2)

    def _robot_position(self, robot_state):
        if robot_state.pose is None:
            return None
        p = robot_state.pose.pose.pose.position
        return p.x, p.y

    def update_swarm(self, event):
        if rospy.is_shutdown():
            return

        # Check task completion and assign new tasks
        for robot_name, state in self.robots.items():
            if state.pose is None:
                continue

            # Check if current task is completed
            if state.current_task_idx is not None:
                tx, ty = self.task_points[state.current_task_idx]
                dist = self._distance(state.pose, tx, ty)
                if dist <= self.goal_radius:
                    self.task_states[state.current_task_idx] = "completed"
                    rospy.loginfo("%s completed task %d", robot_name, state.current_task_idx)
                    state.current_task_idx = None

            # Assign new task if idle
            if state.current_task_idx is None:
                task_idx = self._select_task_for_robot(state)
                if task_idx is not None:
                    self._send_goal(robot_name, task_idx)

        # Basic spacing check (logs near-collision events)
        self._check_spacing()

    def _select_task_for_robot(self, state):
        # Prefer tasks that are unassigned and nearby
        best_idx = None
        best_dist = None

        if state.pose is None:
            return None

        px = state.pose.pose.pose.position.x
        py = state.pose.pose.pose.position.y

        for idx, (tx, ty) in enumerate(self.task_points):
            if self.task_states[idx] != "unassigned":
                continue
            dist = math.sqrt((px - tx) ** 2 + (py - ty) ** 2)
            if best_idx is None or dist < best_dist:
                best_idx = idx
                best_dist = dist

        if best_idx is not None:
            self.task_states[best_idx] = "in_progress"
        return best_idx

    def _send_goal(self, robot_name, task_idx):
        tx, ty = self.task_points[task_idx]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        pub = self.goal_publishers.get(robot_name)
        if pub is not None:
            pub.publish(msg)
            self.robots[robot_name].current_task_idx = task_idx
            rospy.loginfo("Assigned task %d (%.2f, %.2f) to %s", task_idx, tx, ty, robot_name)

    def _check_spacing(self):
        # Log near-collision events based on min_spacing
        robot_names = list(self.robots.keys())
        for i in range(len(robot_names)):
            for j in range(i + 1, len(robot_names)):
                r1 = self.robots[robot_names[i]]
                r2 = self.robots[robot_names[j]]
                if r1.pose is None or r2.pose is None:
                    continue
                x1, y1 = self._robot_position(r1)
                x2, y2 = self._robot_position(r2)
                if x1 is None or x2 is None:
                    continue
                d = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                if d < self.min_spacing:
                    rospy.logwarn(
                        "Near-collision: %s and %s within %.2f m (d = %.2f)",
                        r1.name,
                        r2.name,
                        self.min_spacing,
                        d,
                    )


if __name__ == "__main__":
    try:
        node = SwarmCoordinatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
