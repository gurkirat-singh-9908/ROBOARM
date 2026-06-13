# Contrl – Control Scripts

Current-generation kinematics and Arduino firmware for the 6-DOF robotic arm.

## Folder structure

| File | Purpose |
|------|---------|
| `main.py` | Analytical inverse kinematics (graphical / geometric method, radians) |
| `guidliness.md` / `claudeik.py` | Alternate DH-parameter IK solvers |
| `anayliticalchat.py` / `anayliticalgrok.py` / `chtik.py` | Additional IK derivations (experimental) |
| `dh.py` | Denavit–Hartenberg forward-kinematics helper |
| `robot_visuals.py` | 3-D robot visualizer (matplotlib) – `RobotVisualizer` class |
| `rotation_matrix.py` | Rotation matrix and frame utilities |
| `matrix_multiplyer.py` | Symbolic DH chain (SymPy) |
| `workenvelope.py` | Work-envelope / reachability sweep |
| `controller.py` | Serial controller that sends joint angles to Arduino |
| `sliders.py` | Live slider GUI for manual joint control |
| `88.py` | Quick FK test / scratch file |
| `t2.py` | Scratch / test |
| `test.py` | Unit-level sanity checks |
| `contrl.ino` | Arduino sketch (stub – needs implementation) |
| `reciver.ino` | Arduino receiver sketch (baud 115200, listens to bridge) |
| `rec/rec.ino` | Current Arduino sketch: parses 8-value packet, validates checksum, drives 6 servos via `writeMicroseconds()` + DC gripper (H-bridge IN1=12/IN2=13, signed-ms pulse). Honors `X`/`G` e-stop control bytes (brake gripper mid-pulse + freeze / resume) |

## Robot parameters

| Parameter | Value |
|-----------|-------|
| DOF | 6 + gripper |
| Link lengths (cm) | `[20.5, 28, 28.5, 4, 3.25, 20]` |
| Serial protocol | `j0 j1 j2 j3 j4 j5 gripper checksum\n` at **115 200 baud** |

### Servo calibration

| Servo | Pin | Min µs | Max µs |
|-------|-----|--------|--------|
| s1 (35 kg)  | 2 | 500 | 2400 |
| s2 (150 kg) | 3 | 500 | 2300 |
| s3 (150 kg) | 4 | 725 | 2050 |
| s4 (35 kg)  | 5 | 500 | 2400 |
| s5 (15 kg)  | 6 | 575 | 1900 |
| s6 (15 kg)  | 7 | 575 | 1900 |

## Units convention
All angles are in **radians** internally. Use `c2r` (degrees→radians) and `c2d` (radians→degrees) helpers.

## Running
```bash
python main.py        # run IK solver
python sliders.py     # open slider GUI
python controller.py  # connect to Arduino over serial
```
