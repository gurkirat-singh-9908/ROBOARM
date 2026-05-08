"""ArUco marker pose (camera frame) → IK target pose (base frame).

Closes the autonomous pipeline:

  /aruco/pose  (camera_1 frame, from aruco_detect)
       │
       │   tf2: lookup base_link → camera_1   (provided by publish_calibration)
       ▼
  /target_pose  (base_link frame, consumed by ik_mover)

Optional behaviour:
  • hover_offset_z   — raise the published target above the marker (m) so
                       the gripper hovers instead of crashing into it.
  • target_marker_id — only republish a specific marker ID. -1 = any.
  • smooth_alpha     — exponential smoothing on the published pose
                       (0 = pass-through, 1 = no update). Helps damp jitter
                       from the per-frame solvePnP estimate.

Default frames match the URDF: base_link / camera_1.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose_stamped


class ArucoToTarget(Node):

    def __init__(self):
        super().__init__('aruco_to_target')

        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('camera_frame',      'camera_1')
        self.declare_parameter('aruco_pose_topic',  '/aruco/pose')
        self.declare_parameter('aruco_id_topic',    '/aruco/id')
        self.declare_parameter('target_topic',      '/target_pose')
        self.declare_parameter('target_marker_id',  -1)
        self.declare_parameter('hover_offset_z',    0.05)   # metres above the marker
        self.declare_parameter('smooth_alpha',      0.0)    # 0 = no smoothing
        self.declare_parameter('publish_rate',      10.0)   # Hz; 0 = on every input

        self._base_frame   = self.get_parameter('base_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._target_id    = int(self.get_parameter('target_marker_id').value)
        self._hover_z      = float(self.get_parameter('hover_offset_z').value)
        self._alpha        = float(self.get_parameter('smooth_alpha').value)
        rate               = float(self.get_parameter('publish_rate').value)

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            PoseStamped, self.get_parameter('target_topic').value, 1)

        self._latest_pose: PoseStamped | None = None
        self._latest_id:   int | None = None
        self._smoothed:    PoseStamped | None = None

        self.create_subscription(
            PoseStamped, self.get_parameter('aruco_pose_topic').value,
            self._on_pose, 10)
        self.create_subscription(
            Int32, self.get_parameter('aruco_id_topic').value,
            self._on_id, 10)

        if rate > 0.0:
            self.create_timer(1.0 / rate, self._tick)
            self._on_input_emit = False
        else:
            self._on_input_emit = True

        self.get_logger().info(
            f"aruco_to_target: {self._camera_frame} → {self._base_frame}  "
            f"id_filter={self._target_id}  hover_z={self._hover_z:.3f}m  "
            f"alpha={self._alpha:.2f}  rate={rate}Hz"
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_id(self, msg: Int32):
        self._latest_id = int(msg.data)

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = msg
        if self._on_input_emit:
            self._emit()

    def _tick(self):
        self._emit()

    # ── core ──────────────────────────────────────────────────────────────────

    def _emit(self):
        pose = self._latest_pose
        if pose is None:
            return

        if self._target_id >= 0 and self._latest_id is not None and self._latest_id != self._target_id:
            return

        # solvePnP fills frame_id with 'camera_1'; trust msg.header but fall back to param.
        src_frame = pose.header.frame_id or self._camera_frame

        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                src_frame,
                rclpy.time.Time(),                 # latest available
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF {self._base_frame} ← {src_frame} unavailable: {exc}",
                throttle_duration_sec=2.0)
            return

        out: PoseStamped = do_transform_pose_stamped(pose, tf)
        out.pose.position.z += self._hover_z

        if 0.0 < self._alpha < 1.0 and self._smoothed is not None:
            sp = self._smoothed.pose.position
            so = self._smoothed.pose.orientation
            np_ = out.pose.position
            no  = out.pose.orientation
            a = self._alpha
            np_.x = a * sp.x + (1 - a) * np_.x
            np_.y = a * sp.y + (1 - a) * np_.y
            np_.z = a * sp.z + (1 - a) * np_.z
            no.x  = a * so.x + (1 - a) * no.x
            no.y  = a * so.y + (1 - a) * no.y
            no.z  = a * so.z + (1 - a) * no.z
            no.w  = a * so.w + (1 - a) * no.w

        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self._base_frame

        self._smoothed = out
        self._pub.publish(out)

        p = out.pose.position
        self.get_logger().info(
            f"target_pose (base_link): x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f}",
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ArucoToTarget()
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
