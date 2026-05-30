"""Broadcast the hand-eye calibration result as a static TF.

Reads the YAML produced by compute_calibration and publishes a static
TransformStamped parent_frame → child_frame.
"""
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _matrix_to_quat(R: np.ndarray):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class PublishCalibration(Node):

    def __init__(self):
        super().__init__('handeye_publisher')
        self.declare_parameter('calibration_file',
                               os.path.expanduser('~/handeye_result.yaml'))
        path = os.path.expanduser(self.get_parameter('calibration_file').value)

        if not os.path.exists(path):
            self.get_logger().fatal(f"Calibration file not found: {path}")
            raise RuntimeError("missing calibration file")

        with open(path, 'r') as f:
            cal = yaml.safe_load(f)

        T = np.array(cal['matrix'], dtype=np.float64)
        qx, qy, qz, qw = _matrix_to_quat(T[:3, :3])

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = cal['parent_frame']
        tf.child_frame_id = cal['child_frame']
        tf.transform.translation.x = float(T[0, 3])
        tf.transform.translation.y = float(T[1, 3])
        tf.transform.translation.z = float(T[2, 3])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._br = StaticTransformBroadcaster(self)
        self._br.sendTransform(tf)

        self.get_logger().info(
            f"Broadcasting static TF {cal['parent_frame']} → {cal['child_frame']}  "
            f"t=[{T[0,3]:.4f} {T[1,3]:.4f} {T[2,3]:.4f}]")


def main():
    rclpy.init()
    try:
        node = PublishCalibration()
    except RuntimeError:
        rclpy.shutdown()
        sys.exit(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.ok() guards against rclpy's built-in SIGINT handler having
        # already shut down the context, which would otherwise raise
        # `rcl_shutdown already called on the given context`.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
