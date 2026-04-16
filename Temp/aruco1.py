import cv2
import numpy as np

# -------- SETTINGS --------
font = cv2.FONT_HERSHEY_PLAIN
marker_size = 0.05  # meters (5 cm marker)

# Load camera calibration (IMPORTANT)
# Replace with your calibration file
camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.zeros((5, 1))  # assume no distortion (replace later)

# ArUco dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(dictionary)

# Camera
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:

        # Estimate pose
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):

            corner = corners[i].reshape(4, 2).astype(int)

            # Draw marker
            cv2.polylines(frame, [corner], True, (0, 255, 255), 2)

            # -------- CENTER --------
            cx = int(np.mean(corner[:, 0]))
            cy = int(np.mean(corner[:, 1]))

            # -------- POSE --------
            tvec = tvecs[i][0]  # x, y, z in meters
            rvec = rvecs[i][0]

            # Draw axis
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            # -------- DISPLAY --------
            text = f"ID:{ids[i][0]} X:{tvec[0]:.2f} Y:{tvec[1]:.2f} Z:{tvec[2]:.2f}"
            cv2.putText(frame, text, (cx, cy), font, 1.2, (255, 0, 255), 2)

            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    
    frame_big = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    cv2.imshow("ArUco Pose", frame_big)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
