"""Simulated ArUco publisher for software-only hand-eye calibration.

Reads the TF transform camera_frame → target_frame and publishes it as a fake
/aruco/pose, exactly as the real ArUco detector would during a hardware run.

For eye-in-hand setups (camera on end-effector), target_frame should be a
WORLD-FIXED virtual marker frame (e.g. 'fake_marker'), published by a
static_transform_publisher attached to base_link. As the arm moves, the
camera → marker transform changes, giving varied calibration samples.

For eye-to-hand setups (camera fixed in world, marker on gripper),
target_frame should be gripper_1.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration


class SimArucoPublisher(Node):

    def __init__(self):
        super().__init__('sim_aruco_publisher')

        self.declare_parameter('camera_frame', 'camera_1')
        self.declare_parameter('target_frame', 'fake_marker')
        self.declare_parameter('publish_rate', 10.0)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._target_frame = self.get_parameter('target_frame').value
        rate = self.get_parameter('publish_rate').value

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Sim ArUco publisher: {self._camera_frame} → {self._target_frame} "
            f"@ {rate} Hz on /aruco/pose"
        )

    def _publish(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._camera_frame,
                self._target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        t = tf.transform.translation

        # Only publish when marker is in FRONT of the camera (z > 0 in camera frame).
        # Without this, samples are collected even when camera points away from marker,
        # which corrupts the AX=XB solver with physically impossible observations.
        if t.z <= 0.05:
            self.get_logger().debug(
                f"Marker behind camera (z={t.z:.3f}m) — not publishing",
                throttle_duration_sec=2.0
            )
            return

        r = tf.transform.rotation
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_frame
        msg.pose.position.x    = t.x
        msg.pose.position.y    = t.y
        msg.pose.position.z    = t.z
        msg.pose.orientation.x = r.x
        msg.pose.orientation.y = r.y
        msg.pose.orientation.z = r.z
        msg.pose.orientation.w = r.w

        self._pub.publish(msg)
        self.get_logger().info(
            f"Marker visible at z={t.z:.3f}m — ready to capture (press ENTER)",
            throttle_duration_sec=2.0
        )


def main():
    rclpy.init()
    node = SimArucoPublisher()
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
