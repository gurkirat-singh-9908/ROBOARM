# ROBOARM — Repo Reference (Claude)

Internal map for fast navigation. Authoritative READMEs live per-folder; this is the index + gotchas.

## What it is
Custom **6-DOF servo arm + DC-motor gripper**, Arduino Mega hardware, ROS2 Jazzy on a Raspberry Pi, with three control generations and an active hand-eye calibration effort.

## Control pipeline (current/active)
```
IK / MoveIt2 / manual_joint_commander
        │ /joint_states (sensor_msgs/JointState, radians)
        │ /roboarm/gripper (std_msgs/Float64, open % 0-100)
        ▼
arduino_bridge/bridge_node.py   (host side, applies mechanical offsets + checksum)
        │ UART 115200  "j0 j1 j2 j3 j4 j5 grip checksum\n"
        ▼
Arduino Mega  Contrl/rec/rec.ino  (writeMicroseconds, validates checksum)
        ▼
s1..s6 servos + DC-motor gripper
```

## Directory map
| Path | Role | Status |
|------|------|--------|
| `Contrl/` | Active IK solvers, visualizer, Arduino firmware (`rec/rec.ino` = current sketch) | active |
| `ManualPhase1/ROBO_ws/` | **Current ROS2 Jazzy workspace** — build here w/ `colcon build` | active |
| `ManualPhase1/WebBasedIKV2/` | V2 Flask web control (Pi), `website_dev/app.py` | reference/secondary |
| `vision/` | HSV colour-detection scripts | utility |
| `vision/aruco_prototypes/` | Standalone ArUco prototypes (superseded by ArUcoMarkerRos) | scratch |
| `docs/` | REPO_REFERENCE, CHANGELOG, TASK, AUDIT, PI_SETUP | reference |
| `archive/legacy-website/` | Legacy Flask + external JS robot-sim references | frozen |
| `archive/manualphase1-history/` | V1 slider GUI, old ws snapshot, test sketch | frozen |
| `yolo/` | venv only, no detection script yet | stub |
| `pkg/` | `installed_packages.txt` snapshot of roboenv | reference |
| `roboenv/`, `yolo/yoloenv/` | local venvs (gitignored content) | env |

> Active deployment paths (`Contrl/`, `ManualPhase1/ROBO_ws/`, `ManualPhase1/WebBasedIKV2/`)
> are intentionally NOT renamed — the Pi clones to `~/ROBOARM/...`, `bridge_node.py` hardcodes
> the ws path for `gripper_state.txt`, and `start_roboarm.sh` depends on the web path.

## ROBO_ws packages
| Package | Purpose | Key entry points |
|---------|---------|------------------|
| `arduino_bridge` | `/joint_states`+`/roboarm/gripper` → serial | `bridge_node` |
| `ArUcoMarkerRos` | camera node + ArUco detect | `camera_node.py`, `aruco_detect.py` |
| `roboticarm_description` | URDF/Xacro, meshes, RViz, launch | `display.launch.py`, `gazebo.launch.py` |
| `roboticarm_moveit2` | MoveIt2 config (SRDF, kinematics, controllers) | `demo.launch.py`, `ik_pipeline.launch.py`, `move_group.launch.py` |
| `task1` | high-level tasks; **Task 1 = pick tomato** (conf gate ≥0.75) | `tomato_detector`, `pick_tomato` (+ generic demo: `object_detector`, `visual_servo`, `joint_tracker`, `moveit_interface`) |
| `handeye_calibration` | eye-in-hand calibration workflow | `sample_collector`, `compute_calibration`, `publish_calibration`, `manual_joint_commander`, `aruco_to_target`, `sim_aruco_publisher`, `residual_filter` |

## Serial protocol
```
TX (bridge→Arduino):  "j0 j1 j2 j3 j4 j5 gripper checksum\n"   (j0..j5 servo deg 0-180)
RX (Arduino→bridge):  "OK:j0,j1,j2,j3,j4,j5 grip=<g>"   |   "ERR:checksum_fail(...)" | "ERR:bad_parse(...)"
checksum = (j0+j1+j2+j3+j4+j5+gripper) & 0xFFFF
Control bytes (out-of-band): TX "X\n"=e-stop (brake gripper now, freeze), "G\n"=release.  RX "ESTOP"/"RESUME".
```
- Field [6] (gripper) = **signed pulse-duration ms**: sign=direction, magnitude=run-time (DC-motor, no feedback). Host owns last-commanded %, persisted to `arduino_bridge/gripper_state.txt`.
- Gripper convention: `f`=close, `b`=open (after servo→motor swap). See memory `gripper_direction`.
- `rec/rec.ino` drives the gripper via H-bridge IN1=12/IN2=13 (non-blocking ms pulse, timed brake) and honors the `X`/`G` e-stop bytes. Bridge sends `X` on `/roboarm/estop`=True.

## Hardware / servo calibration
Arduino Mega. Angles radians internally (`c2r`/`c2d` helpers in Contrl).

| Servo | Pin | Min µs | Max µs |
|-------|-----|--------|--------|
| s1 base (35kg)  | 2 | 500 | 2400 |
| s2 shoulder (150kg) | 3 | 500 | 2300 |
| s3 elbow (150kg) | 4 | 725 | 2050 |
| s4 wrist-a (35kg) | 5 | 500 | 2400 |
| s5 wrist-b (15kg) | 6 | 575 | 1900 |
| s6 wrist-c (15kg) | 7 | 575 | 1900 |

Link lengths (cm): `[20.5, 28, 28.5, 4, 3.25, 20]`.
**URDF joint names** (bridge `JOINT_ORDER`, s1..s6): `Revolute 43..48`.

## bridge_node params
`serial_port` (default `/dev/ttyUSB0`), `baud_rate` (115200), `min_send_interval` (0.05s / 20Hz cap). Has 3s reconnect watchdog, jump-threshold warnings, clamps to [0,180].

## Active task: hand-eye calibration (see TASK.md)
Goal: `T_gripper_camera` (phone IP-webcam on gripper, eye-in-hand) → ArUco pose to `base_link` for IK.
**Split decided**: Pi captures `(image, joint_angles)` only — NO cv2 on Pi (apt opencv 4.6 aruco bindings segfault on aarch64). PC runs aruco detect + URDF FK + `cv2.calibrateHandEye(PARK)`.
- Phone: Android IP Webcam, `http://<ip>:8080/video` (MJPEG) or `/shot.jpg`.
- TODO: Pi `sample_capture` node (no cv2), PC `solve_handeye.py`, chessboard intrinsics calib.
- Open risk: URDF zero-pose vs servo zero-pose offsets in bridge (`deg[0]=180-deg[0]` etc.) — verify FK matches physical pose or calibration is garbage.

## Known gotchas / open issues
- `ros2_control` JTC won't load (on_init segfault) after 2026-05-08 apt upgrade; fix = downgrade ros2_control to 2026-01-28 snapshot. See memory `ros2_control_jtc_segfault`.
- Pi cv2 4.6 aruco broken on aarch64 (segfault) — detection moved to PC.
- Joint 4/5 angle correction in bridge negates angle; may stick at 0° after clamp — needs tuning post-wiring.
- `Contrl/contrl.ino` is an empty stub.
- YOLO is env-only, no script.

## Build / run
```bash
source /opt/ros/jazzy/setup.bash
cd ManualPhase1/ROBO_ws && colcon build && source install/setup.bash
ros2 launch roboticarm_description display.launch.py
ros2 run arduino_bridge bridge_node --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200
```
Web: `cd ManualPhase1/WebBasedIKV2/website_dev && python app.py` → `http://<pi-ip>:5000`.
`start_roboarm.sh`: recreates roboenv, starts ngrok tunnel + web app on :8080.
Multi-machine ROS: `ROS_DOMAIN_ID=10`, `ROS_LOCALHOST_ONLY=0` on both. Remote via Tailscale.

## Audit/log trail (all in `docs/`)
- `AUDIT_2026-04-18.md` — handeye package creation + ROBO_ws cleanup.
- `CHANGELOG.md` — ROS↔Arduino link fixes (baud/checksum/sscanf bugs).
- `TASK.md` — live hand-eye state (most current).
- `PI_SETUP.md` — headless Pi + Tailscale + ROS2 install steps.
