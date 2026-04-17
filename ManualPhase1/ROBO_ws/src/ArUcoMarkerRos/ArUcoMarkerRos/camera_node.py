import rclpy
from rclpy.node import Node

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
        camera_index  (int,    default 0)   — OpenCV device index
        frame_width   (int,    default 640) — requested capture width
        frame_height  (int,    default 480) — requested capture height
        fps           (int,    default 30)  — requested capture FPS
        frame_id      (string, default 'camera_frame') — TF frame name
    """

    def __init__(self):
        super().__init__('camera_node')

        # ── declare parameters ────────────────────────────────────────────────
        self.declare_parameter(
            'camera_index', 1,
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
        cam_idx      = self.get_parameter('camera_index').value
        frame_width  = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        fps          = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        # ── open camera ───────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(cam_idx)

        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"Cannot open camera at index {cam_idx}. "
                "Check the camera_index parameter or USB connection.")
            raise RuntimeError(f"Camera {cam_idx} not available")

        # Request resolution and FPS (camera may not honour all values)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        # Log what we actually got
        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f"Camera {cam_idx} opened — "
            f"resolution {actual_w}x{actual_h} @ {actual_fps:.1f} fps")

        # ── ROS objects ───────────────────────────────────────────────────────
        self.bridge    = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer fires at the requested FPS
        timer_period = 1.0 / fps
        self.timer   = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Publishing on /camera/image_raw at ~{fps} Hz")

    # ── TIMER CALLBACK ────────────────────────────────────────────────────────

    def timer_callback(self):
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
