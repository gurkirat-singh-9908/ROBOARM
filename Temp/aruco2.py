import argparse
import sys
import time
import numpy as np
import cv2
from cv2 import aruco

# ── CONFIG ─────────────────────────────────────────────
CAMERA_SOURCE = 1
ARUCO_DICT = aruco.DICT_4X4_50
DISPLAY_SCALE = 2.0   # scaling factor


# ── CAMERA DEFAULT (if no calibration) ─────────────────
def default_camera_matrix(frame_w, frame_h):
    focal = max(frame_w, frame_h)
    cx, cy = frame_w / 2.0, frame_h / 2.0

    K = np.array([[focal, 0, cx],
                  [0, focal, cy],
                  [0, 0, 1]], dtype=np.float64)

    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist


# ── ROTATION → EULER ───────────────────────────────────
def rvec_to_euler(rvec):
    R, _ = cv2.Rodrigues(rvec)

    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll  = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw   = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw   = 0.0

    return roll, pitch, yaw


# ── ARUCO DETECTOR CLASS ───────────────────────────────
class ArucoDetector:
    def __init__(self, marker_length=0.05, camera_matrix=None, dist_coeffs=None):

        aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)

        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.detector = aruco.ArucoDetector(aruco_dict, params)
        self.marker_length = marker_length
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        half = marker_length / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

    def detect(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        results = []
        if ids is None:
            return results

        h, w = frame.shape[:2]
        K = self.camera_matrix
        D = self.dist_coeffs

        if K is None:
            K, D = default_camera_matrix(w, h)

        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            center = c.mean(axis=0)

            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, c, K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                continue

            roll, pitch, yaw = rvec_to_euler(rvec)

            results.append({
                "id": int(marker_id),
                "corners": c,
                "center": center,
                "tvec": tvec.flatten(),
                "rvec": rvec.flatten(),
                "euler": (roll, pitch, yaw)
            })

        return results

    def draw(self, frame, results):

        out = frame.copy()
        h, w = frame.shape[:2]

        K = self.camera_matrix
        D = self.dist_coeffs

        if K is None:
            K, D = default_camera_matrix(w, h)

        for r in results:

            c = r["corners"].astype(int)
            cx, cy = int(r["center"][0]), int(r["center"][1])

            x, y, z = r["tvec"]
            roll, pitch, yaw = r["euler"]

            # Marker border
            cv2.polylines(out, [c], True, (0, 255, 0), 2)

            # Axis
            cv2.drawFrameAxes(out, K, D,
                              r["rvec"].reshape(3, 1),
                              r["tvec"].reshape(3, 1),
                              self.marker_length * 0.6)

            # Text
            cv2.putText(out, f"ID:{r['id']}", (cx, cy - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.putText(out, f"x:{x:.2f} y:{y:.2f} z:{z:.2f}",
                        (cx, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            cv2.putText(out, f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}",
                        (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Center dot
            cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)

        return out


# ── MAIN LOOP ──────────────────────────────────────────
def run():

    cap = cv2.VideoCapture(CAMERA_SOURCE)

    if not cap.isOpened():
        sys.exit("Camera not working")

    detector = ArucoDetector()

    # Resizable window
    cv2.namedWindow("ArUco Detector", cv2.WINDOW_NORMAL)

    while True:

        ret, frame = cap.read()
        if not ret:
            continue

        results = detector.detect(frame)
        out = detector.draw(frame, results)

        h, w = out.shape[:2]

        # 🔥 KEY CHANGE (your requirement)
        target_w = max(960, int(w * DISPLAY_SCALE))
        target_h = max(480, int(h * DISPLAY_SCALE))

        display = cv2.resize(out, (target_w, target_h))

        cv2.imshow("ArUco Detector", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ── ENTRY ──────────────────────────────────────────────
if __name__ == "__main__":
    run()
