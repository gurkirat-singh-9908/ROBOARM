import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('show_feed', True)
        cam_idx = self.get_parameter('camera_index').value
        self._show_feed = self.get_parameter('show_feed').value

        self.pub = self.create_publisher(Point, "/object_pixel", 10)

        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"Cannot open camera at index {cam_idx}. "
                "Override with --ros-args -p camera_index:=<n>.")
            raise RuntimeError(f"Camera {cam_idx} not available")

        self.timer = self.create_timer(0.03, self.loop)

        # HSV bounds for the tracked colour (blue block, default).
        self.lower = np.array([100, 123, 66])
        self.upper = np.array([112, 184, 90])

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            if self._show_feed:
                cv2.imshow("camera", frame)
                cv2.waitKey(1)
            return

        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        msg = Point()
        msg.x = float(cx)
        msg.y = float(cy)
        self.pub.publish(msg)

        if self._show_feed:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.imshow("camera", frame)
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
        node = ObjectDetector()
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
