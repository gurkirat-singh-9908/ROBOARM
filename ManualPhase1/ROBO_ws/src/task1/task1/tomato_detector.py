"""
tomato_detector.py  —  red-tomato detector with a confidence score

Task-1 vision front end. Grabs frames from a camera, segments red regions
in HSV (red wraps the hue axis, so two bands are OR-ed), takes the largest
blob, and scores how tomato-like it is.

Publishes (every frame, so downstream can detect loss-of-target):
  /tomato/center      geometry_msgs/Point    x,y = blob centre px, z = radius px
  /tomato/confidence  std_msgs/Float64       0.0 .. 1.0

Confidence is a classical heuristic — NO neural net is involved (no model
weights / torch in this repo). It blends:
  • circularity  = 4·π·area / perimeter²   (1.0 for a perfect circle)
  • fill         = area / enclosing-circle-area
A ripe tomato reads as a round, solid, well-filled red blob, so both terms
sit near 1.0. Below `min_area` px the score is forced to 0. Swap this node
for a YOLO detector later without touching the picker — the topic contract
(center + confidence) stays the same.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import cv2
import numpy as np


class TomatoDetector(Node):

    def __init__(self):
        super().__init__('tomato_detector')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('show_feed', True)
        self.declare_parameter('min_area', 600)          # px², reject specks
        self.declare_parameter('blur_ksize', 5)          # odd; 0 disables
        # Red lives at both ends of the hue circle → two bands.
        self.declare_parameter('lower1', [0, 120, 70])
        self.declare_parameter('upper1', [10, 255, 255])
        self.declare_parameter('lower2', [170, 120, 70])
        self.declare_parameter('upper2', [180, 255, 255])

        cam_idx = self.get_parameter('camera_index').value
        self._show_feed = bool(self.get_parameter('show_feed').value)
        self._min_area = int(self.get_parameter('min_area').value)
        self._blur = int(self.get_parameter('blur_ksize').value)
        self._lower1 = np.array(self.get_parameter('lower1').value, dtype=np.uint8)
        self._upper1 = np.array(self.get_parameter('upper1').value, dtype=np.uint8)
        self._lower2 = np.array(self.get_parameter('lower2').value, dtype=np.uint8)
        self._upper2 = np.array(self.get_parameter('upper2').value, dtype=np.uint8)

        self.center_pub = self.create_publisher(Point, '/tomato/center', 10)
        self.conf_pub = self.create_publisher(Float64, '/tomato/confidence', 10)

        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().fatal(
                f'Cannot open camera at index {cam_idx}. '
                'Override with --ros-args -p camera_index:=<n>.')
            raise RuntimeError(f'Camera {cam_idx} not available')

        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.timer = self.create_timer(0.03, self.loop)   # ~30 Hz
        self.get_logger().info(
            'tomato_detector up — publishing /tomato/center + /tomato/confidence')

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

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
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
        conf_msg = Float64()
        conf_msg.data = confidence
        self.conf_pub.publish(conf_msg)

        if center is not None:
            p = Point()
            p.x = float(center[0])
            p.y = float(center[1])
            p.z = float(radius)
            self.center_pub.publish(p)

        if self._show_feed:
            if center is not None:
                colour = (0, 255, 0) if confidence >= 0.75 else (0, 165, 255)
                cv2.circle(frame, center, int(radius), colour, 2)
                cv2.circle(frame, center, 4, colour, -1)
                cv2.putText(frame, f'conf {confidence:.2f}',
                            (center[0] - 40, center[1] - int(radius) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            cv2.imshow('tomato_detector', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        if self._show_feed:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = TomatoDetector()
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
