# ROBOARM — Repo Reference (Claude)

Internal map for fast navigation. Authoritative READMEs live per-folder; this is the index + gotchas.

## What it is
Custom **6-DOF servo arm + DC-motor gripper**, Arduino Mega hardware, ROS2 Jazzy on a Raspberry Pi. The runtime is organised as a **pipeline of swappable elements**.

## The element pipeline
```
location  →  ik  →  bridge  →  arm

location   any source of object position. Sub-elements:
             detector  — aruco_source (3D) | color_source (pixel) | YOLO later
             finder    — sweeps the workspace until a detector sees the object
             to_target — reframes a source pose to base_link → /target_pose
ik         /target_pose → ik_mover (setFromIK) → /arm_controller/joint_trajectory
             → traj_interpolator → /joint_states     (NO ros2_control / JTC)
bridge     /joint_states + /roboarm/gripper → UART packet → Arduino → servos+gripper
```
The source is pluggable (ArUco today, colour now, YOLO/web/stored later) — the
contract below never changes.

## Topic contract
```
/camera/image_raw     sensor_msgs/Image
/aruco/pose           geometry_msgs/PoseStamped   metric 3D (camera frame)
/location/center      geometry_msgs/Point         x,y px, z radius px
/location/confidence  std_msgs/Float64            0..1
/location/detected    std_msgs/Bool               conf >= pick_threshold
/target_pose          geometry_msgs/PoseStamped   base_link → ik_mover
/joint_states         sensor_msgs/JointState      radians → bridge
/roboarm/gripper      std_msgs/Float64            open % (100=open, 0=closed)
/roboarm/estop        std_msgs/Bool               global kill (True=KILL)
```

## ROBO_ws packages (`ManualPhase1/ROBO_ws/src/`)
| Package | Element | Key entry points |
|---------|---------|------------------|
| `location` | **location** | `camera`, `aruco_source`, `color_source`, `to_target`, `finder`, `tune` |
| `roboticarm_moveit2` | **ik** | `ik_mover` (C++), `traj_interpolator`, `pose_publisher`; launch `ik.launch.py` |
| `arduino_bridge` | **bridge** | `bridge_node` |
| `tasks` | task orchestrators | `pick` (object-agnostic), `estop` |
| `roboticarm_description` | URDF/Xacro, meshes, RViz | `display.launch.py`, `gazebo.launch.py` |
| `handeye_calibration` | eye-in-hand calibration | `sample_collector`, `compute_calibration`, `publish_calibration`, `manual_joint_commander`, `residual_filter`, `sim_aruco_publisher` |

> `to_target` (was `handeye_calibration/aruco_to_target`) and the camera/aruco
> nodes (were `ArUcoMarkerRos`) now live in `location`. `/aruco/*` topic names
> are unchanged so handeye still works. The old `ArUcoMarkerRos` shell is in
> `archive/ros-pkgs/`.

## Data-driven detection
Per-object config lives in `location/objects/<name>.yaml` (HSV bands, min_area,
pick_threshold, real_diameter_m) — **not** in code. `color_source` loads it via
`object_config.py`: user dir `$ROBOARM_OBJECTS` (default `~/.roboarm/objects`) →
packaged `share/location/objects`. Missing → node tells you to run
`ros2 run location tune -p object:=<name>` and idles. Tomato ships as default.

## Directory map
| Path | Role | Status |
|------|------|--------|
| `ManualPhase1/ROBO_ws/` | **Current ROS2 Jazzy workspace** — `colcon build` here | active |
| `ManualPhase1/WebBasedIKV2/` | V2 Flask web control (Pi); **live Arduino firmware** at `data_fetcher/Arduino/Arduino.ino` | reference |
| `Contrl/` | IK solvers + visualizer (Python). `.ino` files here are dead — NOT the live firmware | mixed |
| `docs/` | REPO_REFERENCE, CHANGELOG, TASK, AUDIT, PI_SETUP | reference |
| `archive/ros-pkgs/` | retired ROS pkgs/nodes (`ArUcoMarkerRos`, `task1-demo-nodes`) | frozen |
| `archive/vision/` | old standalone HSV / ArUco prototype scripts (superseded by `location`) | frozen |
| `archive/legacy-website/`, `archive/manualphase1-history/` | old web + V1 GUI | frozen |
| `pkg/` | `installed_packages.txt` snapshot | reference |
| `roboenv/`, `yolo/yoloenv/` | local venvs (gitignored) | env |

## Serial protocol
```
TX (bridge→Arduino):  "j0 j1 j2 j3 j4 j5 gripper checksum\n"   (j0..j5 servo deg 0-180)
RX (Arduino→bridge):  "Packet OK" | "Checksum fail" | "Packet size error"
checksum = (j0+j1+j2+j3+j4+j5+gripper) & 0xFFFF
Control bytes (out-of-band): TX "X\n"=e-stop (brake gripper now, freeze), "G\n"=release.  RX "ESTOP"/"RESUME".
```
- Field [6] (gripper) = **signed pulse-duration ms**: sign=direction, magnitude=run-time (DC-motor, no feedback). Host owns last-commanded %, persisted to `arduino_bridge/gripper_state.txt`.
- Gripper convention: `f`=close, `b`=open. See memory `gripper_direction`.
- Live sketch `data_fetcher/Arduino/Arduino.ino` drives the gripper via H-bridge IN1=12/IN2=13 and honors the `X`/`G` e-stop bytes.

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

## Known gotchas / open issues
- **ros2_control JTC segfault** (on_init crash, after 2026-05-08 apt upgrade): the
  IK path now **sidesteps it** — `ik.launch.py` uses `traj_interpolator` instead
  of the JointTrajectoryController, so no ros2_control is loaded. The old
  `ik_pipeline.launch.py` (with JTC) remains as a fallback and still needs the
  ros2_control downgrade to 2026-01-28. See memory `ros2_control_jtc_segfault`.
- Pi cv2 4.6 aruco broken on aarch64 (segfault) — `aruco_source` needs cv2 ≥ 4.7
  (pip / roboenv); run detection on the PC.
- Colour source gives pixels only (no depth) → open-loop pick path; ArUco source
  gives metric 3D → real IK path.
- Joint 4/5 angle correction in bridge negates angle; may stick at 0° after clamp.

## Build / run
```bash
source /opt/ros/jazzy/setup.bash
cd ManualPhase1/ROBO_ws && colcon build && source install/setup.bash

# colour + open-loop pick (PC); bridge runs on the Pi separately
ros2 launch tasks pick.launch.py object:=tomato

# ArUco + real-IK path
ros2 launch location location.launch.py source:=aruco
ros2 launch roboticarm_moveit2 ik.launch.py
ros2 run tasks pick --ros-args -p positioning:=ik

# kill / resume
ros2 run tasks estop stop   |   ros2 run tasks estop start
```
Multi-machine ROS: `ROS_DOMAIN_ID` equal, `ROS_LOCALHOST_ONLY=0` on PC + Pi.

## Audit/log trail (all in `docs/`)
- `AUDIT_2026-04-18.md` — handeye package creation + ROBO_ws cleanup.
- `CHANGELOG.md` — ROS↔Arduino link fixes.
- `TASK.md` — hand-eye state.
- `PI_SETUP.md` — headless Pi + Tailscale + ROS2 install.
