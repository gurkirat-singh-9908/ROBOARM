"""Hand-eye sample collector.

Subscribes to /aruco/pose (camera → target) and queries TF2 for the
current (base → gripper) transform. When the user types ENTER on stdin,
the current pair is appended to a YAML file.

Collect 10+ poses spread over rotation and translation for a good fit.
"""
import os
import sys
import threading

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def _matrix_from_pose(px, py, pz, qx, qy, qz, qw):
    """Build a 4x4 homogeneous transform from position + quaternion."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        R = np.eye(3)
    else:
        s = 2.0 / n
        wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
        xx, xy, xz = qx * qx * s, qx * qy * s, qx * qz * s
        yy, yz, zz = qy * qy * s, qy * qz * s, qz * qz * s
        R = np.array([
            [1.0 - (yy + zz), xy - wz,         xz + wy],
            [xy + wz,         1.0 - (xx + zz), yz - wx],
            [xz - wy,         yz + wx,         1.0 - (xx + yy)],
        ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (px, py, pz)
    return T


class SampleCollector(Node):

    def __init__(self):
        super().__init__('handeye_sample_collector')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_frame', 'gripper_link')
        self.declare_parameter('samples_file', '/tmp/handeye_samples.yaml')
        self.declare_parameter('target_marker_id', -1)

        self.base_frame = self.get_parameter('base_frame').value
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.samples_file = self.get_parameter('samples_file').value
        self.target_marker_id = self.get_parameter('target_marker_id').value

        self._latest_target_pose = None
        self._samples = []
        self._lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, '/aruco/pose', self._pose_cb, 10)

        if os.path.exists(self.samples_file):
            self._load_existing()

        self.get_logger().info(
            f"Sample collector ready. base={self.base_frame}  "
            f"gripper={self.gripper_frame}  output={self.samples_file}")
        self.get_logger().info(
            "Press ENTER in this terminal to capture a pair. Type 'q' + ENTER to quit.")

        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    def _load_existing(self):
        try:
            with open(self.samples_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            self._samples = data.get('samples', [])
            self.get_logger().info(
                f"Loaded {len(self._samples)} existing samples from {self.samples_file}")
        except Exception as exc:
            self.get_logger().warn(f"Could not read existing samples: {exc}")

    def _pose_cb(self, msg: PoseStamped):
        with self._lock:
            self._latest_target_pose = msg

    def _get_base_gripper(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.gripper_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f"TF lookup {self.base_frame}->{self.gripper_frame} failed: {exc}")
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return _matrix_from_pose(t.x, t.y, t.z, r.x, r.y, r.z, r.w)

    def _capture(self):
        with self._lock:
            pose = self._latest_target_pose

        if pose is None:
            self.get_logger().warn("No /aruco/pose received yet. Aim the camera at the marker.")
            return

        T_bg = self._get_base_gripper()
        if T_bg is None:
            return

        p = pose.pose.position
        q = pose.pose.orientation
        T_ct = _matrix_from_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        sample = {
            'T_base_gripper': T_bg.tolist(),
            'T_camera_target': T_ct.tolist(),
        }
        self._samples.append(sample)
        self._save()

        self.get_logger().info(
            f"Sample {len(self._samples)} captured  "
            f"(cam→target t={np.round(T_ct[:3, 3], 3).tolist()})")

    def _save(self):
        payload = {'samples': self._samples}
        try:
            with open(self.samples_file, 'w') as f:
                yaml.safe_dump(payload, f, default_flow_style=False)
        except Exception as exc:
            self.get_logger().error(f"Failed to write {self.samples_file}: {exc}")

    def _input_loop(self):
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd in ('q', 'quit', 'exit'):
                self.get_logger().info("Quit requested. Shutting down.")
                rclpy.shutdown()
                return
            if cmd in ('d', 'drop') and self._samples:
                dropped = self._samples.pop()
                self._save()
                self.get_logger().info(
                    f"Dropped last sample. {len(self._samples)} remaining. "
                    f"(was t={np.round(np.array(dropped['T_camera_target'])[:3, 3], 3).tolist()})")
                continue
            self._capture()


def main():
    rclpy.init()
    node = SampleCollector()
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
