# ManualPhase1 – Manual-Control Phase

Current ROS2 workspace and active web interface for manual arm control.
Earlier prototypes (V1 slider GUI, old `ws` snapshot, Arduino test sketch)
are frozen under [`archive/manualphase1-history/`](../archive/manualphase1-history/).

## Sub-folders

### `WebBasedIKV2/` – Version 2: Flask web interface
Web-based IK control served from a Raspberry Pi over the local network.
- `website_dev/` – Flask app (`app.py`), templates, static assets
- `data_fetcher/` – IK calculations, serial bridge, Arduino sketches (`mainUno/`, `Arduino/`)
- `requirements.txt` – Python dependencies

### `ROBO_ws/` – Current ROS2 workspace (Jazzy)
Active ROS2 workspace.  Build with `colcon build` from inside this directory.

The workspace is organised as a pipeline of swappable **elements**:
`location → ik → bridge → arm`.

| Package | Element | Description |
|---------|---------|-------------|
| `location` | **location** | Object-position sources (`aruco_source` 3D, `color_source` pixel), `finder` workspace sweep, `to_target`, and the `tune` HSV tuner. Data-driven per-object config under `objects/`. See [`ROBO_ws/src/location/README.md`](ROBO_ws/src/location/README.md) |
| `roboticarm_moveit2` | **ik** | `ik_mover` (direct setFromIK) + `traj_interpolator` (replaces JTC, no ros2_control). `ik.launch.py` runs the whole IK path. |
| `arduino_bridge` | **bridge** | Bridges `/joint_states` + `/roboarm/gripper` to Arduino serial |
| `tasks` | task | Object-agnostic orchestrators: `pick` (Task 1 = `pick object:=tomato`) + `estop`. See [`ROBO_ws/src/tasks/README.md`](ROBO_ws/src/tasks/README.md) |
| `roboticarm_description` | — | URDF/Xacro model, launch files, RViz configs, STL meshes |
| `handeye_calibration` | — | Eye-in-hand calibration (sample capture, solve, publish TF) |

## Quick-start (ROBO_ws)

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
cd ManualPhase1/ROBO_ws
colcon build

# Source workspace
source install/setup.bash

# Launch display + RViz
ros2 launch roboticarm_description display.launch.py

# Run Arduino bridge (override port as needed)
ros2 run arduino_bridge bridge_node --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p baud_rate:=115200
```

## Serial protocol

```
TX  (ROS → Arduino):   "j0 j1 j2 j3 j4 j5 gripper checksum\n"
RX  (Arduino → ROS):   "OK:j0,j1,j2,j3,j4,j5 grip=<g>"  on success
                        "ERR:checksum_fail(...)"           on bad checksum

Control bytes (out-of-band, single char):
TX  "X\n"  e-stop  — brake gripper now (even mid-pulse), freeze
TX  "G\n"  go      — release e-stop
RX  "ESTOP" / "RESUME" acks
```

`j0..j5` are servo angles 0–180. `gripper` is a **signed** DC-motor pulse in
ms (>0 open, <0 close, 0 = no change), not an angle.
`checksum = (j0+j1+j2+j3+j4+j5+gripper) & 0xFFFF`
