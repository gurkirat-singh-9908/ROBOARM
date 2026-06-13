# ROBOARM – 6-DOF Robotic Arm

A custom 6-DOF servo-driven robotic arm with ROS2 integration, inverse kinematics, ArUco marker vision, and a web control interface.

---

## Repository structure

```
ROBOARM/
├── Contrl/             # Active IK solvers, visualizer, and Arduino firmware
├── ManualPhase1/       # Current ROS2 workspace + active web interface
│   ├── ROBO_ws/            – Current ROS2 (Jazzy) workspace
│   └── WebBasedIKV2/       – V2: Flask web interface on Raspberry Pi
├── vision/             # Colour detection + ArUco prototypes
│   └── aruco_prototypes/   – Standalone ArUco scripts (pre-ROS)
├── docs/               # REPO_REFERENCE, CHANGELOG, TASK, AUDIT, PI_SETUP
├── archive/            # Frozen history (reference only)
│   ├── legacy-website/        – Pre-rewrite Flask app + external JS refs
│   └── manualphase1-history/  – V1 slider GUI, old ws snapshot, test sketch
├── pkg/                # installed_packages.txt snapshot
└── start_roboarm.sh    # Quick-start shell script
```

See [`docs/REPO_REFERENCE.md`](docs/REPO_REFERENCE.md) for the full internal map, gotchas, and pipeline detail.

---

## Architecture

```
PC / Raspberry Pi
┌────────────────────────────────────────────────────────┐
│  ROS2 (Jazzy)                                          │
│  ┌──────────────┐   /joint_states   ┌───────────────┐ │
│  │ joint_pub /  │ ─────────────────▶│ arduino_bridge│ │
│  │ task1 node   │                   │   bridge_node │ │
│  └──────────────┘                   └──────┬────────┘ │
│                                            │ UART 115200│
└────────────────────────────────────────────┼───────────┘
                                             ▼
                                      Arduino Mega
                                      rec/rec.ino
                                      (writeMicroseconds)
                                             │
                              ┌──────────────┼──────────────┐
                            s1..s6       gripper        feedback
```

---

## Quick start

### 1. Raspberry Pi / Ubuntu setup

```bash
# Clone the repo
gh auth login
git clone https://github.com/gurkirat-singh-9908/ROBOARM.git
cd ROBOARM

# Install ROS2 Jazzy (base)
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# Install MoveIt2
sudo apt install ros-jazzy-moveit

# Build ROS2 workspace
source /opt/ros/jazzy/setup.bash
cd ManualPhase1/ROBO_ws
colcon build
source install/setup.bash
```

### 2. Run RViz display

```bash
ros2 launch roboticarm_description display.launch.py
```

### 3. Run Arduino bridge

```bash
ros2 run arduino_bridge bridge_node --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p baud_rate:=115200
```

### 4. Web interface (Raspberry Pi)

```bash
cd ManualPhase1/WebBasedIKV2/website_dev
pip install -r requirements.txt
python app.py
# Open http://<pi-ip>:5000
```

### 5. Desktop slider control

```bash
cd Contrl
python sliders.py
```

---

## Hardware

| Component | Spec |
|-----------|------|
| Controller | Arduino Mega |
| Servo 1 (base rotation, 35 kg) | Pin 2, 500–2400 µs |
| Servo 2 (shoulder, 150 kg) | Pin 3, 500–2300 µs |
| Servo 3 (elbow, 150 kg) | Pin 4, 725–2050 µs |
| Servo 4 (wrist pitch, 35 kg) | Pin 5, 500–2400 µs |
| Servo 5 (wrist roll, 15 kg) | Pin 6, 575–1900 µs |
| Servo 6 (end-effector, 15 kg) | Pin 7, 575–1900 µs |
| Camera | USB / CSI (ArUco detection) |

---

## Robot kinematics (DH parameters)

| Joint | θ offset | d (cm) | a (cm) | α (rad) |
|-------|---------|--------|--------|---------|
| 1 | 0 | 20.5 | 0 | +π/2 |
| 2 | +π/2 | 0 | 28 | 0 |
| 3 | 0 | 0 | 0 | +π/2 |
| 4 | 0 | 32.5 | 0 | −π/2 |
| 5 | 0 | 0 | 0 | +π/2 |
| 6 | 0 | 23.25 | 0 | 0 |

---

## Remote access (Tailscale)

```bash
# Install on Pi
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# Connect from Windows
ssh <user>@<tailscale-ip>
```

Set `ROS_DOMAIN_ID=10` and `ROS_LOCALHOST_ONLY=0` on both machines to share ROS topics.

---

## Known issues / roadmap

- `Contrl/contrl.ino` stub needs implementation for direct hardware control.
- Joint 4/5 angle correction in `bridge_node.py` may need tuning after physical wiring verification.
- Hand-eye calibration in progress — see [`docs/TASK.md`](docs/TASK.md) (Pi captures samples, PC solves).
- YOLO integration (`yolo/`) is environment-only; detection script not yet written.

---

## Folder READMEs

Each sub-folder has its own `README.md` with more detail:
- [`Contrl/README.md`](Contrl/README.md)
- [`ManualPhase1/README.md`](ManualPhase1/README.md)
- [`ManualPhase1/WebBasedIKV2/README.md`](ManualPhase1/WebBasedIKV2/README.md)
- [`vision/README.md`](vision/README.md)
- [`vision/aruco_prototypes/README.md`](vision/aruco_prototypes/README.md)
- [`archive/manualphase1-history/SliderBasedIKV1/README.md`](archive/manualphase1-history/SliderBasedIKV1/README.md)
- [`archive/legacy-website/README.md`](archive/legacy-website/README.md)
