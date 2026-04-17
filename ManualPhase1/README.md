# ManualPhase1 – Manual-Control Phase

Development history and current ROS2 workspace for manual arm control.

## Sub-folders

### `SliderBasedIKV1/` – Version 1: Slider-based IK
First working prototype. Slider GUI on desktop sends joint angles to Arduino over USB serial.
- `contrl/` – Python IK scripts (older snapshots; active versions are in `Contrl/` at the repo root)
- `gui/` – Arduino `gui.ino` sketch + `updateServos` helpers
- `misc/` – Early `controller.py` and `reciver.ino`

### `WebBasedIKV2/` – Version 2: Flask web interface
Web-based IK control served from a Raspberry Pi over the local network.
- `website_dev/` – Flask app (`app.py`), templates, static assets
- `data_fetcher/` – IK calculations, serial bridge, Arduino sketches (`mainUno/`, `Arduino/`)
- `requirements.txt` – Python dependencies

### `ROBO_ws/` – Current ROS2 workspace (Jazzy)
Active ROS2 workspace.  Build with `colcon build` from inside this directory.

| Package | Description |
|---------|-------------|
| `arduino_bridge` | ROS2 node that bridges `/joint_states` topic to Arduino serial |
| `ArUcoMarkerRos` | Camera node + ArUco marker detection |
| `roboticarm_description` | URDF/Xacro model, launch files, RViz configs, STL meshes |
| `roboticarm_moveit2` | MoveIt2 configuration (SRDF, kinematics, controllers) |
| `task1` | High-level task node (visual servo, object detection, MoveIt interface) |

### `ws/` – Earlier ROS2 workspace snapshot
Near-identical to `ROBO_ws/` with minor differences in `fake_camera.py` and `setup.py`.  Kept for reference.

### `test/` – Arduino GUI test
Standalone `gui.ino` + `updateServos` test sketch.

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
```

All values are integers (microseconds for servo writeMicroseconds).
`checksum = (j0+j1+j2+j3+j4+j5+gripper) & 0xFFFF`
