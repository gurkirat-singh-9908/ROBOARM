"""
tune.py  —  interactive HSV tuner for the colour source

This is the "set up thresholding when the data is missing" flow. Point the
camera at the object, drag the trackbars until the mask isolates it cleanly,
then press ``s`` to save ``objects/<object>.yaml`` (to the writable user dir,
``~/.roboarm/objects`` by default). ``color_source`` then loads it by name.

Run:
  ros2 run location tune --ros-args -p object:=tomato -p camera_index:=0

Keys:   s = save    q / Esc = quit without saving

Two HSV bands are exposed because some colours (red) wrap the hue axis. If your
object needs only one band, set band-2 low > high so it matches nothing.
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from location import object_config


def _seed_from_existing(name):
    """Pre-load trackbars from an existing config if one is present."""
    try:
        return object_config.load(name)
    except object_config.MissingObjectConfig:
        return None


def main():
    rclpy.init()
    node = Node('tune')
    node.declare_parameter('object', 'tomato')
    node.declare_parameter('camera_index', 0)
    name = node.get_parameter('object').value
    cam_idx = node.get_parameter('camera_index').value

    # V4L2 first, fall back to the default backend (virtual cams like Iriun
    # reject an explicit CAP_V4L2 open) — same as the camera node.
    cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        node.get_logger().fatal(
            f'Cannot open camera index {cam_idx}. Try -p camera_index:=1 or 2.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    win = f'tune:{name}'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 480)

    seed = _seed_from_existing(name) or {}
    d1l = seed.get('lower1', [0, 120, 70]);  d1u = seed.get('upper1', [10, 255, 255])
    d2l = seed.get('lower2', [170, 120, 70]); d2u = seed.get('upper2', [180, 255, 255])

    def add(label, val, maxv):
        cv2.createTrackbar(label, win, int(val), maxv, lambda _v: None)

    # Band 1
    add('H1 lo', d1l[0], 179); add('H1 hi', d1u[0], 179)
    add('S1 lo', d1l[1], 255); add('S1 hi', d1u[1], 255)
    add('V1 lo', d1l[2], 255); add('V1 hi', d1u[2], 255)
    # Band 2
    add('H2 lo', d2l[0], 179); add('H2 hi', d2u[0], 179)
    add('S2 lo', d2l[1], 255); add('S2 hi', d2u[1], 255)
    add('V2 lo', d2l[2], 255); add('V2 hi', d2u[2], 255)
    # Shape
    add('min_area', int(seed.get('min_area', 600)), 10000)
    add('blur(odd)', int(seed.get('blur_ksize', 5)), 31)

    def g(label):
        return cv2.getTrackbarPos(label, win)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    node.get_logger().info(f"Tuning '{name}'.  s=save  q/Esc=quit")

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret or frame is None:
            # Keep the GUI responsive even with no frames (a silent `continue`
            # leaves the window unpainted and looking dead). Warn, pump keys.
            node.get_logger().warn(
                f'No frame from camera index {cam_idx} — is it streaming?',
                throttle_duration_sec=2.0)
            if (cv2.waitKey(30) & 0xFF) in (ord('q'), 27):
                break
            continue

        blur = g('blur(odd)')
        src = frame
        if blur >= 3 and blur % 2 == 1:
            src = cv2.GaussianBlur(frame, (blur, blur), 0)
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

        l1 = np.array([g('H1 lo'), g('S1 lo'), g('V1 lo')], np.uint8)
        u1 = np.array([g('H1 hi'), g('S1 hi'), g('V1 hi')], np.uint8)
        l2 = np.array([g('H2 lo'), g('S2 lo'), g('V2 lo')], np.uint8)
        u2 = np.array([g('H2 hi'), g('S2 hi'), g('V2 hi')], np.uint8)

        mask = cv2.bitwise_or(cv2.inRange(hsv, l1, u1), cv2.inRange(hsv, l2, u2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        min_area = g('min_area')
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        overlay = frame.copy()
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) >= min_area:
                (x, y), r = cv2.minEnclosingCircle(c)
                cv2.circle(overlay, (int(x), int(y)), int(r), (0, 255, 0), 2)

        masked = cv2.bitwise_and(frame, frame, mask=mask)
        # Label the panes: LEFT = live feed + detected circle, RIGHT = the HSV
        # mask applied (only in-range pixels show; tune until your object lights
        # up white/colour here).
        cv2.putText(overlay, "LIVE", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(masked, "MASK", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        view = np.hstack([overlay, masked])
        cv2.putText(view, "s=save  q/Esc=quit", (10, view.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(win, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            node.get_logger().info('Quit without saving.')
            break
        if key == ord('s'):
            cfg = {
                'object': name,
                'lower1': [int(g('H1 lo')), int(g('S1 lo')), int(g('V1 lo'))],
                'upper1': [int(g('H1 hi')), int(g('S1 hi')), int(g('V1 hi'))],
                'lower2': [int(g('H2 lo')), int(g('S2 lo')), int(g('V2 lo'))],
                'upper2': [int(g('H2 hi')), int(g('S2 hi')), int(g('V2 hi'))],
                'min_area': int(min_area),
                'blur_ksize': int(blur),
                'pick_threshold': float(seed.get('pick_threshold', 0.75)),
                'real_diameter_m': float(seed.get('real_diameter_m', 0.06)),
            }
            path = object_config.save(name, cfg)
            node.get_logger().info(f'Saved {path}')

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
