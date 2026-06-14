"""
color_source.py  —  colour-threshold object-position source

A location *source*: it answers "where is the object in the image and how sure
am I". It is object-agnostic — what the object looks like (HSV bands, area
floor, threshold, real size) comes from ``objects/<object>.yaml`` loaded via
``object_config``. No object is hard-coded; swap this whole node for a YOLO
source later without changing the contract below.

Pipeline: grab frame → HSV two-band mask (colour may wrap the hue axis) →
morphology → largest blob → confidence from circularity × fill.

Published contract (every frame, so a finder/picker can detect loss):
  /location/center      geometry_msgs/Point    x,y = blob centre px, z = radius px
  /location/confidence  std_msgs/Float64       0.0 .. 1.0
  /location/detected    std_msgs/Bool          confidence >= pick_threshold

If the object has no config the node logs how to tune it and idles (publishes
detected=False) instead of crashing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

from location import object_config


class ColorSource(Node):

    def __init__(self):
        super().__init__('color_source')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('object', 'tomato')      # which objects/<name>.yaml
        self.declare_parameter('show_feed', True)
        self.declare_parameter('image_topic', '/camera/image_raw')

        self._object = self.get_parameter('object').value
        self._show_feed = bool(self.get_parameter('show_feed').value)
        image_topic = self.get_parameter('image_topic').value

        # Publishers come first so we can emit detected=False while idle.
        self.center_pub = self.create_publisher(Point, '/location/center', 10)
        self.conf_pub = self.create_publisher(Float64, '/location/confidence', 10)
        self.detected_pub = self.create_publisher(Bool, '/location/detected', 10)

        # ── Object config (data-driven thresholding) ────────────────────────
        try:
            self._cfg = object_config.load(self._object)
            self.get_logger().info(
                f"Loaded config for '{self._object}' from "
                f"{object_config.config_path(self._object)}")
        except object_config.MissingObjectConfig as exc:
            # Idle, don't crash: a finder/picker just sees detected=False.
            self.get_logger().error(str(exc))
            self._idle_timer = self.create_timer(0.5, self._idle)
            return

        self._load_cfg(self._cfg)

        # Frames come from the shared `camera` node, not a direct device open —
        # so color_source, aruco_source and anything else share one camera.
        self._bridge = CvBridge()
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.get_logger().info(
            f"color_source up for '{self._object}' — subscribing {image_topic}, "
            "publishing /location/*")

    # ── Config ───────────────────────────────────────────────────────────────

    def _load_cfg(self, cfg: dict):
        self._min_area = int(cfg.get('min_area', 600))
        self._blur = int(cfg.get('blur_ksize', 5))
        self._threshold = float(cfg.get('pick_threshold', 0.75))
        self._lower1 = np.array(cfg['lower1'], dtype=np.uint8)
        self._upper1 = np.array(cfg['upper1'], dtype=np.uint8)
        self._lower2 = np.array(cfg['lower2'], dtype=np.uint8)
        self._upper2 = np.array(cfg['upper2'], dtype=np.uint8)

    def _idle(self):
        """No config: keep telling downstream there is no target."""
        d = Bool(); d.data = False
        self.detected_pub.publish(d)

    # ── Detection ───────────────────────────────────────────────────────────

    def _score(self, contour):
        """Return (confidence 0..1, (cx, cy), radius) for one contour."""
        area = cv2.contourArea(contour)
        if area < self._min_area:
            return 0.0, None, 0.0

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return 0.0, None, 0.0

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        (x, y), radius = cv2.minEnclosingCircle(contour)
        circle_area = np.pi * radius * radius
        fill = area / circle_area if circle_area > 0 else 0.0

        confidence = float(np.clip(0.5 * circularity + 0.5 * fill, 0.0, 1.0))
        return confidence, (int(x), int(y)), float(radius)

    def on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}',
                                    throttle_duration_sec=2.0)
            return

        hsv = frame
        if self._blur >= 3 and self._blur % 2 == 1:
            hsv = cv2.GaussianBlur(frame, (self._blur, self._blur), 0)
        hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)

        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._lower1, self._upper1),
            cv2.inRange(hsv, self._lower2, self._upper2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        confidence, center, radius = 0.0, None, 0.0
        if contours:
            c = max(contours, key=cv2.contourArea)
            confidence, center, radius = self._score(c)

        # Always publish — confidence 0 with no centre means "no target".
        conf_msg = Float64(); conf_msg.data = confidence
        self.conf_pub.publish(conf_msg)
        det_msg = Bool(); det_msg.data = confidence >= self._threshold
        self.detected_pub.publish(det_msg)

        if center is not None:
            p = Point()
            p.x = float(center[0]); p.y = float(center[1]); p.z = float(radius)
            self.center_pub.publish(p)

        if self._show_feed:
            if center is not None:
                colour = (0, 255, 0) if confidence >= self._threshold else (0, 165, 255)
                cv2.circle(frame, center, int(radius), colour, 2)
                cv2.circle(frame, center, 4, colour, -1)
                cv2.putText(frame, f'{self._object} {confidence:.2f}',
                            (center[0] - 40, center[1] - int(radius) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            cv2.imshow('color_source', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self._show_feed:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = ColorSource()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
