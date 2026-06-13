# Hand-Eye Calibration — Task State

## Goal

Solve **eye-in-hand** calibration for the RoboArm: find the rigid transform
`T_gripper_camera` that relates the phone camera (mounted on the gripper)
to the gripper link of the URDF.

Output: a YAML/static-TF that lets downstream nodes turn an ArUco pose in
camera frame into a target in `base_link` for IK.

## Architecture shift (this session)

Pi-only execution failed: Ubuntu Noble's apt `python3-opencv` 4.6 has
broken aruco bindings on aarch64 (segfault on `aruco.detectMarkers` and on
`DetectorParameters` property access). Rebuilding under the venv with
opencv-python 4.9 still hit the same import path inside the colcon-built
entry point.

**New split:**

| Side | Responsibility |
|------|----------------|
| **Pi** | Drive servos via `arduino_bridge`, publish `/joint_states`, grab MJPEG frames from the phone IP webcam, save `(image, joint_angles)` pair per sample. **No** ArUco detection, **no** calibrateHandEye on the Pi. |
| **PC (x86 / Mac / etc.)** | Pull saved samples (or live-stream them), run `cv2.aruco` detection, compute URDF forward kinematics, run `cv2.calibrateHandEye`, emit `T_gripper_camera` as YAML. |

Final step pushes that YAML back to the Pi; `publish_calibration` already
exists and broadcasts it as a static TF.

## Capture flow (per sample)

1. On Pi terminal A: `manual_joint_commander` — type joint angles in degrees,
   commands publish to `/joint_states`, `arduino_bridge` drives the arm,
   `robot_state_publisher` updates TF.
2. Operator visually verifies the marker is in view.
3. On Pi terminal B: `sample_capture` (TODO) — press ENTER, node grabs
   one MJPEG frame from `http://<phone-ip>:8080/shot.jpg` and the current
   `/joint_states`, writes to `samples/NN_image.png` and
   `samples/NN_joints.yaml`.
4. Repeat 10–20 times across varied poses (≥ 30° rotational diversity is
   what `calibrateHandEye` needs to be well-conditioned).

## Solve flow (PC)

1. `rsync` the `samples/` dir from Pi to PC.
2. `solve_handeye.py` (TODO) on PC:
   - For each sample: aruco-detect → solvePnP → `T_camera_target`.
   - For each sample: load URDF + joint angles → FK to `T_base_gripper`.
   - Feed two stacks into `cv2.calibrateHandEye(method=PARK)`.
   - Emit `handeye_result.yaml` with `T_gripper_camera`.
3. `scp handeye_result.yaml` back to Pi.

## Progress

### Done

- [x] `handeye_calibration` ROS2 package with sample_collector,
  compute_calibration, publish_calibration nodes (works in eye_in_hand
  mode).
- [x] `arduino_bridge` driving servos on `/dev/ttyUSB0` from `/joint_states`.
- [x] `manual_joint_commander.py` interactive stdin → `/joint_states`
  publisher (deg/rad input, 6 joints, 20 Hz hold).
- [x] `camera_node` extended to accept HTTP/RTSP URL via `source` param,
  FFMPEG vs V4L2 backend auto-pick.
- [x] `aruco_detect` node API guarded against cv2 4.6 vs 4.7+ split (legacy
  `detectMarkers` fallback + `ArucoDetector` when available, subpix setter
  guarded by `hasattr`).
- [x] Launch file restructured to omit stdin-conflicting nodes.
- [x] Phone IP Webcam stream verified live on Pi:
  `http://192.168.29.81:8080/video` returns `multipart/x-mixed-replace`,
  camera_node opens at 1920x1080@25, publishes `/camera/image_raw` ~6 Hz.
- [x] Workspace builds with cv-bridge + image-transport apt installed.

### Blocked

- [ ] aruco_detect runtime — `[ros2run]: Segmentation fault` on Pi.
  Cause: apt cv2 4.6 aruco bindings broken on aarch64. Rebuilding under
  venv (cv2 4.9) did not resolve because cv_bridge boost extension
  loads system cv2 4.6 first, ABI conflict.
  **Decision: stop fighting Pi cv2. Move detection to PC.**

### TODO (this branch)

- [ ] Pi-side `sample_capture` node — minimal: HTTP GET `shot.jpg` + grab
  current `/joint_states`, write paired files. No cv2 dependency on Pi.
- [ ] PC-side `solve_handeye.py` — pure-Python: aruco + urdfpy/yourdfpy FK
  + `cv2.calibrateHandEye`.
- [ ] Glue: rsync recipe in README, `publish_calibration` already wired.

### Open questions

- Camera intrinsics. Right now we fall back to a default fx=fy=max(w,h),
  cx=w/2 cy=h/2 matrix. Pose accuracy will be poor. Need a quick chessboard
  intrinsic calibration on PC before solving extrinsics. Add to TODO.
- URDF zero-pose vs servo zero-pose: the bridge does
  `deg[0] = 180 - deg[0]` and similar offsets. Verify each commanded
  angle in `/joint_states` lands on the same physical pose the URDF
  predicts. If not, the FK chain will be wrong and calibration garbage.

## Files of note

```
ManualPhase1/ROBO_ws/src/
├── ArUcoMarkerRos/
│   └── ArUcoMarkerRos/
│       ├── camera_node.py          # URL/index source param (modified)
│       └── aruco_detect.py         # 4.6/4.7 API guard (modified)
├── arduino_bridge/                 # existing, unchanged
├── handeye_calibration/
│   ├── config/handeye_config.yaml  # mode=eye_in_hand, camera_source set (modified)
│   ├── launch/handeye_capture.launch.py  # restructured (modified)
│   ├── handeye_calibration/
│   │   ├── sample_collector.py     # existing, ROS-based capture
│   │   ├── compute_calibration.py  # existing, calibrateHandEye solver
│   │   ├── publish_calibration.py  # existing, static TF broadcaster
│   │   └── manual_joint_commander.py  # NEW
│   └── setup.py                    # entry point added (modified)
└── roboticarm_description/         # URDF (unchanged)
```

## Phone setup (reference)

- Android **IP Webcam** by Pavel Khlebovich.
- LAN URL: `http://192.168.29.81:8080/`
- MJPEG stream: `/video`
- Single-shot JPEG: `/shot.jpg`  ← preferred for sample_capture (no decoder
  warnings, easier on slow wifi).
