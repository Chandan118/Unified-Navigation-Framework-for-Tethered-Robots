#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import threading
import math

# Import standard ROS 2 message types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArduinoInterfaceNode(Node):
    """
    This node handles serial communication with an Arduino board.
    It subscribes to /cmd_vel to send motor commands and publishes sensor
    data and wheel odometry from the Arduino.
    """
    def __init__(self):
        super().__init__('arduino_interface_node')

        # --- Parameters for Serial Connection ---
        self.declare_parameter('serial_port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('command_timeout_sec', 0.6)
        self.declare_parameter('enable_sensor_queries', False)
        self.declare_parameter('publish_fallback_odom', True)
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.command_timeout_sec = (
            self.get_parameter('command_timeout_sec').get_parameter_value().double_value
        )
        self.enable_sensor_queries = (
            self.get_parameter('enable_sensor_queries').get_parameter_value().bool_value
        )
        self.publish_fallback_odom = (
            self.get_parameter('publish_fallback_odom').get_parameter_value().bool_value
        )

        # --- Initialize Serial Connection ---
        try:
            self.arduino = serial.Serial(port, baud, timeout=1)
            # Keep startup fast so odom is available before Nav2 activation.
            time.sleep(0.4)
            self.get_logger().info(f"Successfully connected to Arduino on port {port}.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            return
        
        # A threading lock is crucial to prevent multiple threads (callbacks)
        # from trying to use the serial port at the same time.
        self.serial_lock = threading.Lock()

        # --- ROS 2 Communication ---
        # Subscriber for motor velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for all sensor data from Arduino
        self.ultrasonic_pub = self.create_publisher(Range, 'sensor/ultrasonic', 10)
        self.mq2_pub = self.create_publisher(Float32, 'sensor/mq2', 10)
        # TODO: Add publishers for your MQ135 and CO2 sensors if needed
        
        # Publisher for wheel odometry
        # IMPORTANT: This requires your Arduino to be sending encoder data.
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_update = self.get_clock().now()

        # --- Timers for Periodic Tasks ---
        # A timer to periodically read and publish sensor data
        self.sensor_timer = self.create_timer(0.1, self.sensor_timer_callback)
        self.safety_timer = self.create_timer(0.1, self.safety_timer_callback)
        self.odom_timer = self.create_timer(0.05, self.odom_timer_callback)

        self.get_logger().info("Arduino Interface Node has started.")

    def cmd_vel_callback(self, msg: Twist):
        """Callback for /cmd_vel topic to control motors."""
        self.last_cmd_time = self.get_clock().now()
        command = b''
        linear = msg.linear.x
        angular = msg.angular.z
        self.cmd_linear = float(linear)
        self.cmd_angular = float(angular)

        if abs(angular) > 0.18 and abs(linear) < 0.08:
            command = b'L' if angular > 0.0 else b'R'
        elif linear > 0.05:
            command = b'F'  # Forward
        elif linear < -0.05:
            command = b'B'  # Backward
        elif angular > 0.1:
            command = b'L'  # Turn Left (needs to be implemented in Arduino sketch)
        elif angular < -0.1:
            command = b'R'  # Turn Right (needs to be implemented in Arduino sketch)
        else:
            command = b'S'  # Stop

        if command:
            with self.serial_lock:
                self.arduino.write(command)

    def safety_timer_callback(self):
        """Hard stop when velocity command stream is stale."""
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.command_timeout_sec and self.arduino.is_open:
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0
            with self.serial_lock:
                self.arduino.write(b'S')

    def sensor_timer_callback(self):
        """Periodically queries sensors and publishes their data."""
        if not self.enable_sensor_queries:
            return
        self.read_and_publish_sensor('U', self.ultrasonic_pub, 'Range')
        self.read_and_publish_sensor('M', self.mq2_pub, 'Float32')
        # TODO: Add more calls for other sensors
        # self.read_and_publish_sensor('C', self.mq135_pub, 'Float32')
        # self.read_and_publish_sensor('O', self.co2_pub, 'Float32')

    def read_and_publish_sensor(self, command, publisher, msg_type):
        """Helper function to query a sensor and publish its data."""
        try:
            with self.serial_lock:
                self.arduino.write(command.encode())
                # A short delay might be needed for the Arduino to process and respond
                time.sleep(0.1) 
                response = self.arduino.readline().decode('utf-8', errors='ignore').strip()

            if response:
                value = float(response)
                
                if msg_type == 'Range':
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'ultrasonic_link' # Must match a link in your URDF
                    msg.radiation_type = Range.ULTRASOUND
                    msg.field_of_view = 0.5  # radians, ~28 degrees
                    msg.min_range = 0.02   # meters
                    msg.max_range = 4.0    # meters
                    msg.range = value / 100.0  # Convert cm from Arduino to meters
                    publisher.publish(msg)

                elif msg_type == 'Float32':
                    msg = Float32()
                    msg.data = value
                    publisher.publish(msg)
            else:
                self.get_logger().warn(f"No data received from Arduino for command '{command}'")
        except (ValueError, serial.SerialException, UnicodeDecodeError) as e:
            self.get_logger().error(f"Error processing sensor data for command '{command}': {e}")

    def odom_timer_callback(self):
        """Publish fallback odometry integrated from commanded velocity."""
        if not self.publish_fallback_odom:
            return

        now = self.get_clock().now()
        dt = (now - self.last_odom_update).nanoseconds / 1e9
        self.last_odom_update = now
        if dt <= 0.0 or dt > 0.3:
            return

        self.odom_x += self.cmd_linear * math.cos(self.odom_yaw) * dt
        self.odom_y += self.cmd_linear * math.sin(self.odom_yaw) * dt
        self.odom_yaw += self.cmd_angular * dt

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.orientation.z = math.sin(self.odom_yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.odom_yaw * 0.5)
        msg.twist.twist.linear.x = self.cmd_linear
        msg.twist.twist.angular.z = self.cmd_angular
        msg.pose.covariance[0] = 0.2
        msg.pose.covariance[7] = 0.2
        msg.pose.covariance[35] = 0.4
        msg.twist.covariance[0] = 0.2
        msg.twist.covariance[35] = 0.3
        self.odom_pub.publish(msg)

    def on_shutdown(self):
        """Cleanup function called on node shutdown."""
        self.get_logger().info("Shutting down, stopping motors and closing serial port.")
        if self.arduino.is_open:
            with self.serial_lock:
                self.arduino.write(b'S') # Ensure motors are stopped
                self.arduino.close()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()