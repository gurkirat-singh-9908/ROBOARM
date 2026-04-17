import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
import numpy as np

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)  # match topic!
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        now = self.get_clock().now().to_msg()

        # --- Image ---
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = now
        msg.header.frame_id = 'camera_1'   # ← YOUR camera TF frame
        self.img_pub.publish(msg)

        # --- CameraInfo ---
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = 'camera_1'  # ← SAME frame, this is the key!
        info.width = 640
        info.height = 480

        # Simple pinhole intrinsics (tune fx, fy, cx, cy to your real camera later)
        fx, fy = 600.0, 600.0
        cx, cy = 320.0, 240.0
        info.k = [
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0
        ]
        info.p = [
            fx,  0.0, cx, 0.0,
            0.0, fy,  cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.info_pub.publish(info)

def main():
    rclpy.init()
    node = FakeCamera()
    rclpy.spin(node)
    rclpy.shutdown()
