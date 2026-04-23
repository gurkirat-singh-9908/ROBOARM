import rclpy
from rclpy.node import Node

import cv2
from cv2 import aruco
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge


# ── CONSTANTS ──────────────────────────────────────────────────────────────────
ARUCO_DICT    = aruco.DICT_4X4_50
MARKER_LENGTH = 0.05          # metres  — change to your physical marker size


# ── HELPERS ────────────────────────────────────────────────────────────────────

def default_camera_matrix(frame_w: int, frame_h: int):
    """Fallback intrinsics when no calibration file is available."""
    focal  = float(max(frame_w, frame_h))
    cx, cy = frame_w / 2.0, frame_h / 2.0
    K = np.array([[focal, 0.0,   cx],
                  [0.0,   focal, cy],
                  [0.0,   0.0,   1.0]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist


def rvec_to_quaternion(rvec: np.ndarray):
    """Convert OpenCV rotation vector → (qx, qy, qz, qw) as plain Python floats.

    Pure numpy + OpenCV implementation — no scipy dependency.
    Uses the standard rotation-matrix-to-quaternion formula (Shepperd method).
    All outputs are cast to plain float for ROS2 message field compatibility
    with numpy >= 2.0 (which removed implicit np.float64 → float coercion).
    """
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))

    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s  = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s  = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s  = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return float(qx), float(qy), float(qz), float(qw)


# ── NODE ───────────────────────────────────────────────────────────────────────

class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # ── detector setup ──────────────────────────────────────────────────
        aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        params     = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(aruco_dict, params)

        # marker corner template in marker frame  (z = 0 plane)
        half = MARKER_LENGTH / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        # ── show_feed parameter ──────────────────────────────────────────────
        # Pass --ros-args -p show_feed:=true to open a live detection window.
        # Stays False by default so the node runs headless in production.
        self.declare_parameter('show_feed', False)
        self._show_feed = self.get_parameter('show_feed').value

        if self._show_feed:
            cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
            self.get_logger().info("show_feed=True — live detection window enabled")

        # cached intrinsics — filled on first frame
        self._K    = None
        self._dist = None

        self.bridge = CvBridge()

        # ── ROS subscriber ───────────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # ── ROS publishers ───────────────────────────────────────────────────
        # Full 6-DOF pose  (position in metres, orientation as quaternion)
        self.pub_pose   = self.create_publisher(PoseStamped, '/aruco/pose',         10)
        # Pixel centre of the detected marker
        self.pub_pixel  = self.create_publisher(Point,       '/aruco/pixel_center', 10)
        # Integer marker ID
        self.pub_id     = self.create_publisher(Int32,       '/aruco/id',           10)
        # Physical side length of the marker (metres)
        self.pub_size   = self.create_publisher(Float32,     '/aruco/size',         10)

        self.get_logger().info("ArUco detector node started — listening on /camera/image_raw")


    # ── CALLBACK ───────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        """Process one frame and publish results for every detected marker."""

        # Convert ROS Image → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        h, w = frame.shape[:2]

        # Build / cache intrinsics once we know the image size
        if self._K is None:
            self._K, self._dist = default_camera_matrix(w, h)
            self.get_logger().info(
                f"Using default camera matrix for {w}x{h} — "
                "replace with a calibration file for better accuracy."
            )

        # ── detection ────────────────────────────────────────────────────────
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return   # nothing found this frame

        # ── per-marker processing ─────────────────────────────────────────────
        for i, marker_id in enumerate(ids.flatten()):

            c = corners[i][0]   # shape (4, 2) — pixel corners

            # ── PnP pose estimation ──────────────────────────────────────────
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, c,
                self._K, self._dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                self.get_logger().warn(f"solvePnP failed for marker {marker_id}")
                continue

            tvec = tvec.flatten()           # [x, y, z]  in metres
            rvec = rvec.flatten()           # [rx, ry, rz]

            qx, qy, qz, qw = rvec_to_quaternion(rvec)

            # pixel centre
            cx_px = float(c[:, 0].mean())
            cy_px = float(c[:, 1].mean())

            stamp = msg.header.stamp

            # ── publish PoseStamped ──────────────────────────────────────────
            pose_msg = PoseStamped()
            pose_msg.header.stamp    = stamp
            pose_msg.header.frame_id = 'camera_1'

            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])

            pose_msg.pose.orientation.x = qx   # already plain float
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            self.pub_pose.publish(pose_msg)

            # ── publish pixel centre ─────────────────────────────────────────
            pixel_msg   = Point()
            pixel_msg.x = cx_px
            pixel_msg.y = cy_px
            pixel_msg.z = 0.0
            self.pub_pixel.publish(pixel_msg)

            # ── publish marker ID ────────────────────────────────────────────
            id_msg       = Int32()
            id_msg.data  = int(marker_id)
            self.pub_id.publish(id_msg)

            # ── publish marker size ──────────────────────────────────────────
            size_msg      = Float32()
            size_msg.data = float(MARKER_LENGTH)
            self.pub_size.publish(size_msg)

            self.get_logger().debug(
                f"Marker {marker_id} | "
                f"pos=({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}) m | "
                f"pixel=({cx_px:.0f}, {cy_px:.0f})"
            )

        # ── optional live window ──────────────────────────────────────────────
        if self._show_feed:
            display = self._draw_detections(frame, corners, ids)
            cv2.imshow('ArUco Detection', display)
            # q or ESC closes the window and shuts the node down cleanly
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self.get_logger().info("Feed window closed by user — shutting down.")
                cv2.destroyAllWindows()
                rclpy.shutdown()

    # ── DRAW HELPER ────────────────────────────────────────────────────────────

    def _draw_detections(self, frame, corners, ids):
        """Annotate frame with marker borders, axes, pose text and centre dot."""
        out = frame.copy()

        if ids is None:
            return out

        for i, marker_id in enumerate(ids.flatten()):
            c      = corners[i][0]
            cx_px  = int(c[:, 0].mean())
            cy_px  = int(c[:, 1].mean())

            # ── re-run PnP just for drawing (results already published above) ─
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, c,
                self._K, self._dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            # Marker border
            cv2.polylines(out, [c.astype(int)], True, (0, 255, 0), 2)

            if success:
                tvec_f = tvec.flatten()
                x, y, z = tvec_f[0], tvec_f[1], tvec_f[2]

                # 3-D axes
                cv2.drawFrameAxes(
                    out, self._K, self._dist,
                    rvec, tvec,
                    MARKER_LENGTH * 0.6
                )

                # Position text
                cv2.putText(out,
                    f"x:{x:.2f} y:{y:.2f} z:{z:.2f}",
                    (cx_px, cy_px - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            # ID label
            cv2.putText(out,
                f"ID:{marker_id}",
                (cx_px, cy_px - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Centre dot
            cv2.circle(out, (cx_px, cy_px), 5, (0, 0, 255), -1)

        return out


# ── ENTRY POINT ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
