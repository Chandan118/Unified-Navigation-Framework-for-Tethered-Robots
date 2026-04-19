#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class JetsonCameraPublisher(Node):
    """
    A ROS2 node that captures video from a camera using a GStreamer pipeline
    and publishes it as a ROS2 Image message. Optimized for NVIDIA Jetson platforms.
    """
    def __init__(self):
        super().__init__('jetson_camera_publisher')

        # --- Declare parameters that can be set from the launch file ---
        # The GStreamer pipeline itself
        self.declare_parameter('gstreamer_pipeline', '')
        self.declare_parameter('camera_device', '/dev/video0')
        # The desired publishing frame rate
        self.declare_parameter('frame_rate', 30.0)
        # The topic to publish the images on
        self.declare_parameter('topic_name', 'camera/image_raw')
        # The TF frame_id to stamp on the image messages
        self.declare_parameter('frame_id', 'camera_link')

        # --- Get parameters from the launch file or defaults ---
        pipeline = self.get_parameter('gstreamer_pipeline').get_parameter_value().string_value
        camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- Parameter Validation ---
        self.cap = None
        self.disabled = False
        if not pipeline:
            self.get_logger().fatal("GStreamer pipeline is not set! Camera node disabled.")
            self.disabled = True
            return
            
        # --- ROS 2 Communication Setup ---
        # Create the publisher
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        
        # Create a timer that fires at the specified frame rate to trigger the callback
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # --- OpenCV VideoCapture Initialization ---
        # Prefer V4L2 on this hardware. GStreamer fallback can produce noisy critical logs.
        self.cap = self._open_camera(camera_device, pipeline)
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error(
                "Could not open camera from device or GStreamer pipeline."
            )
            self.disabled = True
            return

        # Initialize the bridge between OpenCV images and ROS Image messages
        self.bridge = CvBridge()
        self.get_logger().info(f"Jetson camera publisher started successfully.")
        self.get_logger().info(f"Publishing to topic '{topic_name}' at {frame_rate} FPS.")

    def timer_callback(self):
        """
        This function is called periodically by the timer.
        It reads a frame from the camera, converts it to a ROS message, and publishes it.
        """
        # Read a frame from the video capture
        if self.disabled or self.cap is None:
            return
        ret, frame = self.cap.read()
        
        if ret:
            # Convert the OpenCV image (BGR8 format) to a ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # Stamp the message with the current time and the correct frame_id
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = self.frame_id
            
            # Publish the message
            self.publisher_.publish(ros_image_msg)
        else:
            self.get_logger().warn("Could not read frame from camera. The GStreamer pipeline may have stalled or the camera was disconnected.")

    def on_shutdown(self):
        """
        This function is called automatically on node shutdown (e.g., Ctrl+C).
        It ensures that the camera resource is properly released.
        """
        self.get_logger().info("Shutting down node, releasing camera resource.")
        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def _open_camera(self, camera_device: str, pipeline: str):
        self.get_logger().info(f"Trying V4L2 camera device: {camera_device}")

        # Try numeric index first, e.g., /dev/video0 -> 0.
        idx = None
        if camera_device.startswith('/dev/video'):
            suffix = camera_device.replace('/dev/video', '')
            if suffix.isdigit():
                idx = int(suffix)
        if idx is not None:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                return cap
            cap.release()

        # Try direct device path with V4L2 backend.
        cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        if cap.isOpened():
            return cap
        cap.release()

        # Last fallback to GStreamer pipeline.
        self.get_logger().warn("V4L2 open failed, trying GStreamer pipeline fallback.")
        self.get_logger().info(f"Pipeline: {pipeline}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            return cap
        cap.release()
        return None


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = JetsonCameraPublisher()
    try:
        if not getattr(camera_publisher, 'disabled', False):
            rclpy.spin(camera_publisher)
        else:
            # Keep node alive to avoid repeated crash/restart loops.
            rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # The on_shutdown method is called before destroying the node
        camera_publisher.on_shutdown()
        camera_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()