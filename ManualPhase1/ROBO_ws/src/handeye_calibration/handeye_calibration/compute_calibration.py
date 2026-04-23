"""Hand-eye calibration solver.

Reads base↔gripper and camera↔target sample pairs from a YAML file and solves
AX = XB with cv2.calibrateHandEye.

Output modes:
  mode = eye_to_hand  → X = T_camera_to_base      (camera fixed in world)
  mode = eye_in_hand  → X = T_camera_to_gripper   (camera on end-effector)

For eye-to-hand we invert the base→gripper samples so the classic AX=XB form
matches the cv2 API convention (R_gripper2base / t_gripper2base inputs).
"""
import os
import sys

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node


_METHOD_MAP = {
    'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
    'PARK':       cv2.CALIB_HAND_EYE_PARK,
    'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def _matrix_to_xyzrpy(T: np.ndarray):
    """Return (x, y, z, roll, pitch, yaw) — for a human-readable log/YAML."""
    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    sy = (T[0, 0] ** 2 + T[1, 0] ** 2) ** 0.5
    singular = sy < 1e-6
    if not singular:
        roll = float(np.arctan2(T[2, 1], T[2, 2]))
        pitch = float(np.arctan2(-T[2, 0], sy))
        yaw = float(np.arctan2(T[1, 0], T[0, 0]))
    else:
        roll = float(np.arctan2(-T[1, 2], T[1, 1]))
        pitch = float(np.arctan2(-T[2, 0], sy))
        yaw = 0.0
    return x, y, z, roll, pitch, yaw


class ComputeCalibration(Node):

    def __init__(self):
        super().__init__('handeye_compute')

        self.declare_parameter('samples_file', '/tmp/handeye_samples.yaml')
        self.declare_parameter('calibration_file',
                               os.path.expanduser('~/handeye_result.yaml'))
        self.declare_parameter('mode', 'eye_to_hand')
        self.declare_parameter('method', 'PARK')
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_frame', 'gripper_1')
        self.declare_parameter('camera_frame', 'camera_1')

    def run(self) -> int:
        p = {k: self.get_parameter(k).value for k in (
            'samples_file', 'calibration_file', 'mode', 'method', 'min_samples',
            'base_frame', 'gripper_frame', 'camera_frame')}

        method_key = str(p['method']).upper()
        if method_key not in _METHOD_MAP:
            self.get_logger().error(
                f"Unknown method '{p['method']}'. Valid: {list(_METHOD_MAP)}")
            return 2
        method_cv = _METHOD_MAP[method_key]

        mode = str(p['mode']).lower()
        if mode not in ('eye_to_hand', 'eye_in_hand'):
            self.get_logger().error(
                f"Unknown mode '{p['mode']}'. Must be eye_to_hand or eye_in_hand.")
            return 2

        try:
            with open(p['samples_file'], 'r') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().error(f"Samples file not found: {p['samples_file']}")
            return 2

        samples = data.get('samples', [])
        n = len(samples)
        if n < max(3, int(p['min_samples'])):
            self.get_logger().error(
                f"Need at least {max(3, int(p['min_samples']))} samples, got {n}.")
            return 2
        if n < 10:
            self.get_logger().warn(
                f"Only {n} samples — ≥10 recommended for stable calibration.")

        R_g2b, t_g2b = [], []   # gripper → base   (cv2 input for eye_in_hand)
        R_t2c, t_t2c = [], []   # target → camera  (cv2 input convention)

        for s in samples:
            T_bg = np.array(s['T_base_gripper'], dtype=np.float64)
            T_ct = np.array(s['T_camera_target'], dtype=np.float64)

            if mode == 'eye_in_hand':
                # gripper2base = T_bg itself
                T_g2b = T_bg
            else:  # eye_to_hand
                # Treat base as the "gripper" surrogate → invert so the arm
                # motion appears as a moving end-effector relative to a fixed
                # camera. cv2 solves for X such that A_i X = X B_i.
                T_g2b = _invert(T_bg)

            # target2cam = inverse of camera2target
            T_t2c = _invert(T_ct)

            R_g2b.append(T_g2b[:3, :3])
            t_g2b.append(T_g2b[:3, 3].reshape(3, 1))
            R_t2c.append(T_t2c[:3, :3])
            t_t2c.append(T_t2c[:3, 3].reshape(3, 1))

        R_X, t_X = cv2.calibrateHandEye(
            R_g2b, t_g2b, R_t2c, t_t2c, method=method_cv)

        X = np.eye(4)
        X[:3, :3] = R_X
        X[:3, 3] = t_X.flatten()

        if mode == 'eye_to_hand':
            # X solves for camera expressed in the base frame.
            parent_frame = p['base_frame']
            result_name = 'T_base_camera'
        else:
            # X solves for camera expressed in the gripper frame.
            parent_frame = p['gripper_frame']
            result_name = 'T_gripper_camera'

        x, y, z, roll, pitch, yaw = _matrix_to_xyzrpy(X)
        out = {
            'mode': mode,
            'method': method_key,
            'num_samples': n,
            'parent_frame': parent_frame,
            'child_frame': p['camera_frame'],
            'translation': {'x': x, 'y': y, 'z': z},
            'rotation_rpy': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'matrix': X.tolist(),
            'result_name': result_name,
        }

        out_path = os.path.expanduser(p['calibration_file'])
        os.makedirs(os.path.dirname(os.path.abspath(out_path)) or '.', exist_ok=True)
        with open(out_path, 'w') as f:
            yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(
            f"Calibration {result_name} ({parent_frame} → {p['camera_frame']})")
        self.get_logger().info(
            f"  t = [{x:.4f} {y:.4f} {z:.4f}] m")
        self.get_logger().info(
            f"  rpy = [{roll:.4f} {pitch:.4f} {yaw:.4f}] rad")
        self.get_logger().info(f"Written to {out_path}")
        return 0


def main():
    rclpy.init()
    node = ComputeCalibration()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
