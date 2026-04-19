"""
PATCH: Add tether collision detection and yielding behavior to robot_agent.py
This implements the Swarm Fuzzy-DRL yielding rules for Task 1.
"""

import math
import random
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg      import Odometry
from std_msgs.msg      import Bool, Float32, String, String

# --- NEW: Swarm tether states subscription ---
from geometry_msgs.msg import Point  # For tether endpoint parsing
import json

# (Existing constants and code remain the same until RobotAgent class)

class RobotAgent(Node):

    def __init__(self):
        super().__init__('robot_agent')

        # ── ROS 2 parameters ──────────────────────────────────────────────────
        self.declare_parameter('robot_id',    0)
        self.declare_parameter('nest_x',      DEFAULT_NEST_X)
        self.declare_parameter('nest_y',      DEFAULT_NEST_Y)
        self.declare_parameter('target_x',    DEFAULT_TARGET_X)
        self.declare_parameter('target_y',    DEFAULT_TARGET_Y)
        self.declare_parameter('max_speed',   DEFAULT_MAX_SPEED)
        self.declare_parameter('sensor_range',DEFAULT_SENSOR_RANGE)
        self.declare_parameter('fault_mode',  False)

        # ── NEW: Swarm yielding parameters ───────────────────────────────────
        self.declare_parameter('yield_enabled', True)
        self.declare_parameter('yield_safety_margin', 0.6)  # meters
        self.declare_parameter('yield_stop_duration', 2.0)  # seconds

        self._id           = self.get_parameter('robot_id').value
        self._nest         = (
            self.get_parameter('nest_x').value,
            self.get_parameter('nest_y').value
        )
        self._target       = (
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value
        )
        self._max_speed    = self.get_parameter('max_speed').value
        self._sensor_range = self.get_parameter('sensor_range').value
        self._fault_mode   = self.get_parameter('fault_mode').value

        # ── Yielding config ───────────────────────────────────────────────────
        self._yield_enabled   = self.get_parameter('yield_enabled').value
        self._safety_margin   = self.get_parameter('yield_safety_margin').value
        self._yield_duration  = self.get_parameter('yield_stop_duration').value

        # ── Internal state ────────────────────────────────────────────────────
        self._x        = self._nest[0] + random.uniform(-0.3, 0.3)
        self._y        = self._nest[1] + random.uniform(-0.3, 0.3)
        self._yaw      = random.uniform(-math.pi, math.pi)
        self._state    = STATE_EXPLORE
        self._carrying = False

        # Yielding state
        self._is_yielding = False
        self._yield_until = 0.0
        self._last_yield_time = 0.0

        # (Rest of existing initialization unchanged...)
        # ACO motion state
        self._theta_prev = self._yaw
        self._levy_steps = 0
        self._levy_total = self._sample_levy()

        self._fault_prob = 0.20 if self._fault_mode else 0.0

        self._successful_forages = 0
        self._total_attempts     = 0
        self._dist_travelled_m   = 0.0
        self._rest_until         = 0.0

        # ── ROS 2 I/O ─────────────────────────────────────────────────────────
        ns = self.get_namespace()   # e.g. /robot_0

        self._cmd_pub    = self.create_publisher(Twist,   f'{ns}/cmd_vel',  10)
        self._state_pub  = self.create_publisher(String,  f'{ns}/agent_state', 10)
        self._forage_pub = self.create_publisher(Float32, '/swarm/forage_count', 10)

        self.create_subscription(Odometry, f'{ns}/odom', self._odom_cb, 10)

        # ── NEW: Subscribe to swarm tether states ─────────────────────────────
        self.create_subscription(
            String,
            '/swarm_tether_states',
            self._tether_states_cb,
            10
        )

        # Control loop at 10 Hz
        self.create_timer(0.10, self._control_loop)

        self.get_logger().info(
            f'Robot {self._id} agent started  '
            f'nest={self._nest}  target={self._target}  '
            f'yield_enabled={self._yield_enabled}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # NEW: Tether states callback
    # ─────────────────────────────────────────────────────────────────────────
    def _tether_states_cb(self, msg: String):
        """
        Parse swarm tether states and decide if yielding is necessary.
        Message format: JSON array of tethers, each: {
          "robot_id": "...",
          "start": [x, y],
          "end": [x, y],
          "thickness": 0.02,
          "active": true
        }
        """
        try:
            import json
            tethers = json.loads(msg.data)

            if not self._yield_enabled or self._is_yielding:
                return

            # Check each tether for crossing
            for tether in tethers:
                if not tether.get('active', False):
                    continue

                start_pt = tether['start']
                end_pt   = tether['end']

                # Check if robot's current path will cross this tether segment
                if self._will_path_cross_tether(start_pt, end_pt):
                    self.get_logger().warn(
                        f'Robot {self._id}: Tether crossing detected from {tether["robot_id"]}. '
                        f'Yielding for {self._yield_duration}s.'
                    )
                    self._is_yielding = True
                    self._yield_until = time.time() + self._yield_duration
                    self._last_yield_time = time.time()
                    break  # Yield for first detected crossing

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().debug(f'Failed to parse tether states: {e}')

    def _will_path_cross_tether(self, tether_start, tether_end):
        """
        Simple collision prediction: will robot's current velocity vector
        intersect the tether line segment within the next 2 seconds?
        """
        # Robot's predicted path segment (current pos + velocity * 2s)
        # Assume max_speed heading (simplified - use actual velocity if available)
        lookahead_distance = self._max_speed * 2.0  # 2 second prediction

        robot_end_x = self._x + math.cos(self._yaw) * lookahead_distance
        robot_end_y = self._y + math.sin(self._yaw) * lookahead_distance

        # Check line segment intersection
        return self._segments_intersect(
            self._x, self._y, robot_end_x, robot_end_y,
            tether_start[0], tether_start[1],
            tether_end[0], tether_end[1]
        )

    def _segments_intersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        """Check if line segments (x1,y1)-(x2,y2) and (x3,y3)-(x4,y4) intersect."""
        def ccw(ax, ay, bx, by, cx, cy):
            return (cy-ay)*(bx-ax) > (by-ay)*(cx-ax)

        # Add safety margin
        margin = self._safety_margin

        # Expand tether segment by margin (simple approximation: grow endpoints)
        # This creates a "collision tube" around the tether
        dx = x4 - x3
        dy = y4 - y3
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            nx = -dy / length * margin
            ny =  dx / length * margin
            # Expand both sides
            x3a, y3a = x3 + nx, y3 + ny
            x3b, y3b = x3 - nx, y3 - ny
            x4a, y4a = x4 + nx, y4 + ny
            x4b, y4b = x4 - nx, y4 - ny

            # Check both boundaries
            if ccw(x1, y1, x3a, y3a, x4a, y4a) == ccw(x2, y2, x3a, y3a, x4a, y4a):
                return False
            if ccw(x1, y1, x3b, y3b, x4b, y4b) == ccw(x2, y2, x3b, y3b, x4b, y4b):
                return False
            if ccw(x1, y1, x2, y2, x3a, y3a) == ccw(x1, y1, x2, y2, x3b, y3b):
                return False
            if ccw(x1, y1, x2, y2, x4a, y4a) == ccw(x1, y1, x2, y2, x4b, y4b):
                return False
            return True

        return False

    # ─────────────────────────────────────────────────────────────────────────
    # MODIFIED: Control loop with yielding logic
    # ─────────────────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        now = time.time()

        # ── Check if yielding period has ended ───────────────────────────────
        if self._is_yielding and now >= self._yield_until:
            self._is_yielding = False
            self.get_logger().info(f'Robot {self._id}: Resuming from yield')

        # ── State: YIELDING override ─────────────────────────────────────────
        if self._is_yielding:
            # Stop completely
            self._pub_cmd(0.0, 0.0)

            # Still publish state for monitoring
            state_msg = String()
            state_msg.data = f'{self._id}|YIELDING|{self._x:.2f}|{self._y:.2f}|{self._successful_forages}|{self._dist_travelled_m:.2f}'
            self._state_pub.publish(state_msg)
            return

        # ── Existing state machine (EXPLORE, HOMING, RETURN, IDLE) ────────────
        # (Keep all original code unchanged from here...)

        if random.random() < self._fault_prob:
            conc, gx, gy = 0.0, 0.0, 0.0
        else:
            conc, gx, gy = PheromoneGridProxy.query(self._x, self._y)

        if self._state == STATE_EXPLORE:
            self._total_attempts += 1
            PheromoneGridProxy.deposit(self._x, self._y, carrying=False)

            dist_to_target = self._dist(self._target)
            if dist_to_target < DETECTION_RADIUS_M:
                self._state    = STATE_HOMING
                self._carrying = True
                self.get_logger().debug(f'Robot {self._id}: TARGET FOUND → HOMING')
                self._pub_cmd(0.0, 0.0)
                return

            cmd = self._explore_step(gx, gy)
            self._cmd_pub.publish(cmd)

        elif self._state == STATE_HOMING:
            PheromoneGridProxy.deposit(self._x, self._y, carrying=True)

            dist_to_nest = self._dist(self._nest)
            if dist_to_nest < NEST_RADIUS_M:
                self._successful_forages += 1
                self._state    = STATE_IDLE
                self._carrying = False
                self._rest_until = now + REST_DURATION_S

                msg = Float32()
                msg.data = float(self._successful_forages)
                self._forage_pub.publish(msg)

                self.get_logger().debug(
                    f'Robot {self._id}: DELIVERED  total={self._successful_forages}'
                )
                self._pub_cmd(0.0, 0.0)
                return

            cmd = self._steer_toward(*self._nest)
            self._cmd_pub.publish(cmd)

        elif self._state == STATE_RETURN:
            dist_to_nest = self._dist(self._nest)
            if dist_to_nest < NEST_RADIUS_M:
                self._state      = STATE_IDLE
                self._rest_until = now + REST_DURATION_S
            else:
                self._cmd_pub.publish(self._steer_toward(*self._nest))

        state_msg = String()
        state_msg.data = (
            f'{self._id}|{self._state}|'
            f'{self._x:.2f}|{self._y:.2f}|'
            f'{self._successful_forages}|{self._dist_travelled_m:.2f}'
        )
        self._state_pub.publish(state_msg)
