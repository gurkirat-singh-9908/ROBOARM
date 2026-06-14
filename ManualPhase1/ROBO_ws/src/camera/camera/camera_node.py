import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor


class CameraNode(Node):
    """
    Reads frames from a USB / built-in webcam and publishes them on
    /camera/image_raw  (sensor_msgs/Image, encoding bgr8).

    ROS2 parameters (can be overridden at launch):
        source        (string, default '0') — OpenCV index ('0', '1', …) OR
                                              a URL ('http://phone-ip:8080/video',
                                              'rtsp://…', '/dev/videoN').
        camera_index  (int,    default 0)   — legacy fallback; used only when
                                              `source` is unset / empty.
        frame_width   (int,    default 640) — requested capture width
        frame_height  (int,    default 480) — requested capture height
        fps           (int,    default 30)  — requested capture FPS
        frame_id      (string, default 'camera_frame') — TF frame name
    """

    def __init__(self):
        super().__init__('camera_node')

        # ── declare parameters ────────────────────────────────────────────────
        # `source` accepts either a numeric index (`0`, `1`) or a URL string,
        # so the parameter is dynamically typed: ROS2 otherwise locks the
        # type to the declared default and `-p source:=0` would be rejected
        # as INTEGER vs STRING.
        self.declare_parameter(
            'source', '',
            ParameterDescriptor(
                description='Numeric index (e.g. "0") or URL (http://, rtsp://). '
                            'Overrides camera_index when non-empty.',
                dynamic_typing=True))
        self.declare_parameter(
            'camera_index', 0,
            ParameterDescriptor(description='OpenCV VideoCapture device index'))
        self.declare_parameter(
            'frame_width', 640,
            ParameterDescriptor(description='Requested capture width in pixels'))
        self.declare_parameter(
            'frame_height', 480,
            ParameterDescriptor(description='Requested capture height in pixels'))
        self.declare_parameter(
            'fps', 30,
            ParameterDescriptor(description='Requested capture frame rate'))
        self.declare_parameter(
            'frame_id', 'camera_frame',
            ParameterDescriptor(description='TF frame name stamped on every image'))

        # ── read parameters ───────────────────────────────────────────────────
        # Use an explicit `is None` check rather than truthiness so that an
        # integer source of `0` (a valid V4L2 index) is preserved instead of
        # being collapsed to an empty string.
        _src_val = self.get_parameter('source').value
        source_str   = '' if _src_val is None else str(_src_val).strip()
        cam_idx      = self.get_parameter('camera_index').value
        frame_width  = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        fps          = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        # Resolve final source: explicit param > legacy index.
        if source_str:
            try:
                resolved = int(source_str)
                is_url = False
            except ValueError:
                resolved = source_str
                is_url = True
        else:
            resolved = int(cam_idx)
            is_url = False

        # ── open camera ───────────────────────────────────────────────────────
        # URL → FFMPEG (handles http/rtsp). Local index → V4L2 (lower latency),
        # but fall back to the default backend: virtual cams (Iriun, OBS,
        # v4l2loopback) often reject an explicit CAP_V4L2 open while opening
        # fine with the auto-selected backend.
        if is_url:
            self.cap = cv2.VideoCapture(resolved, cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(resolved, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().warn(
                    f"V4L2 backend could not open source '{resolved}' — "
                    "retrying with the default backend (virtual camera?).")
                self.cap.release()
                self.cap = cv2.VideoCapture(resolved)

        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"Cannot open camera source '{resolved}'. "
                "Check `source` / `camera_index` parameter or stream URL.")
            raise RuntimeError(f"Camera source {resolved!r} not available")

        if not is_url:
            # MJPG keeps USB bandwidth low → fewer dropped/queued frames than YUYV.
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            # Shrink driver buffer so read() returns the freshest frame.
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            self.cap.set(cv2.CAP_PROP_FPS,          fps)
        else:
            # Network streams: minimal buffer so we read the freshest frame.
            try:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

        self._is_url = is_url

        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS) or float(fps)

        self.get_logger().info(
            f"Camera source {resolved!r} opened — "
            f"resolution {actual_w}x{actual_h} @ {actual_fps:.1f} fps "
            f"(backend={'FFMPEG' if is_url else 'V4L2'})")

        # ── ROS objects ───────────────────────────────────────────────────────
        self.bridge    = CvBridge()

        # Sensor-data QoS: BEST_EFFORT + depth=1 → drop stale frames instead
        # of buffering them. Subscriber must use the same profile to connect.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(Image, '/camera/image_raw', sensor_qos)

        # Timer fires at the requested FPS
        timer_period = 1.0 / fps
        self.timer   = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Publishing on /camera/image_raw at ~{fps} Hz")

    # ── TIMER CALLBACK ────────────────────────────────────────────────────────

    def timer_callback(self):
        # For local cams flush stale frames sitting in the V4L2 queue.
        # For network streams skip flush — the driver doesn't queue them
        # the same way and extra grab() can stall on slow links.
        if not self._is_url:
            for _ in range(2):
                self.cap.grab()

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn(
                "Failed to grab frame — camera disconnected?")
            return

        # Convert OpenCV BGR → ROS Image
        msg                  = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = self.frame_id

        self.publisher.publish(msg)

    # ── CLEANUP ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released.")
        super().destroy_node()


# ── ENTRY POINT ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()

    try:
        node = CameraNode()
    except RuntimeError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
