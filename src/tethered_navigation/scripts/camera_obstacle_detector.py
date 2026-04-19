#!/usr/bin/env python3
import time
from typing import List

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class CameraObstacleDetector(Node):
    """Detects front obstacles from camera feed with a lightweight YOLO model."""

    def __init__(self) -> None:
        super().__init__("camera_obstacle_detector")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("obstacle_topic", "/camera/obstacle")
        self.declare_parameter("score_topic", "/camera/obstacle_score")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("area_ratio_threshold", 0.12)
        self.declare_parameter("process_every_n_frames", 4)
        self.declare_parameter("publish_debug_view", False)
        self.declare_parameter("debug_topic", "/camera/obstacle_debug")

        self.bridge = CvBridge()
        self.model = None
        self.frame_count = 0
        self.last_infer_time = 0.0

        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.area_ratio_threshold = float(self.get_parameter("area_ratio_threshold").value)
        self.process_every_n_frames = int(self.get_parameter("process_every_n_frames").value)

        self.pub_obs = self.create_publisher(Bool, self.get_parameter("obstacle_topic").value, 10)
        self.pub_score = self.create_publisher(Float32, self.get_parameter("score_topic").value, 10)

        self.publish_debug_view = bool(self.get_parameter("publish_debug_view").value)
        if self.publish_debug_view:
            self.pub_debug = self.create_publisher(
                Image, self.get_parameter("debug_topic").value, 5
            )
        else:
            self.pub_debug = None

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.image_callback, 10
        )

        self._load_model()
        self.get_logger().info("Camera obstacle detector started.")

    def _load_model(self) -> None:
        if YOLO is None:
            self.get_logger().warn("ultralytics not available, camera obstacle output will stay false.")
            return
        model_path = self.get_parameter("model_path").value
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model: {model_path}")
        except Exception as exc:
            self.get_logger().warn(f"Could not load model {model_path}: {exc}")

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % max(1, self.process_every_n_frames) != 0:
            return

        if self.model is None:
            self._publish(False, 0.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        h, w = frame.shape[:2]
        frame_area = float(h * w)
        max_score = 0.0
        obstacle = False

        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            verbose=False,
            classes=[0, 1, 2, 3, 5, 7, 15, 16, 56, 57],  # Common obstacle classes.
        )

        if results:
            boxes = results[0].boxes
            for box in boxes:
                conf = float(box.conf.item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                bw = max(0.0, x2 - x1)
                bh = max(0.0, y2 - y1)
                area_ratio = (bw * bh) / frame_area

                # Weighted score: confidence and occupied image area.
                score = conf * min(1.0, area_ratio / self.area_ratio_threshold)
                max_score = max(max_score, score)
                if area_ratio >= self.area_ratio_threshold and conf >= self.conf_threshold:
                    obstacle = True

                if self.pub_debug is not None:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (10, 180, 255), 2)

        self._publish(obstacle, max_score)

        if self.pub_debug is not None:
            dt = time.time() - self.last_infer_time if self.last_infer_time else 0.0
            self.last_infer_time = time.time()
            cv2.putText(
                frame,
                f"obs={obstacle} score={max_score:.2f} dt={dt:.2f}s",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if not obstacle else (0, 80, 255),
                2,
            )
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception:
                pass

    def _publish(self, obstacle: bool, score: float) -> None:
        b = Bool()
        b.data = obstacle
        s = Float32()
        s.data = float(score)
        self.pub_obs.publish(b)
        self.pub_score.publish(s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
