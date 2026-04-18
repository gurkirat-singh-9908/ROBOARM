# handeye_calibration

Eye-to-hand / eye-in-hand calibration for the RoboArm, built on
`cv2.calibrateHandEye`.

## What it does

Solves `AX = XB` for the unknown fixed transform between the camera and the
robot, using pairs of:

- `A` — robot motion (base → gripper) read from the TF tree
- `B` — camera motion (camera → target) read from `/aruco/pose`

`X` is:

| mode          | X                      | meaning                               |
|---------------|------------------------|---------------------------------------|
| `eye_to_hand` | `T_base_camera`        | camera fixed in world, target on gripper |
| `eye_in_hand` | `T_gripper_camera`     | camera on gripper, target fixed in world |

## Workflow

1. **Configure** `config/handeye_config.yaml` — at minimum set `mode` and the
   TF frame names to match your URDF.
2. **Physical setup** — print a DICT_4X4_50 ArUco marker and mount it:
   - `eye_to_hand`: on the end-effector
   - `eye_in_hand`: at a fixed, known location in the workspace
3. **Capture samples**

   ```bash
   source install/setup.bash
   ros2 launch handeye_calibration handeye_capture.launch.py
   ```

   In the `sample_collector` terminal:
   - Move the arm to a new pose, wait for `/aruco/pose` to update.
   - Press `ENTER` to capture. Repeat for ≥10 varied poses.
   - `d` + `ENTER` drops the last sample.
   - `q` + `ENTER` exits.

4. **Solve**

   ```bash
   ros2 run handeye_calibration compute_calibration --ros-args \
       -p samples_file:=/tmp/handeye_samples.yaml \
       -p calibration_file:=$HOME/handeye_result.yaml \
       -p mode:=eye_to_hand \
       -p method:=PARK
   ```

5. **Broadcast**

   ```bash
   ros2 run handeye_calibration publish_calibration --ros-args \
       -p calibration_file:=$HOME/handeye_result.yaml
   ```

## Methods

Selectable via `method:=` — `TSAI`, `PARK` (default), `HORAUD`, `ANDREFF`,
`DANIILIDIS`. PARK and DANIILIDIS are typically the most robust.

## Sample pose guidance

For a stable solution, vary **both** rotation and translation between poses.
Rotations should include all three axes. Purely translational motion makes
the system ill-conditioned and the solver can return bogus results.

## Files

```
handeye_calibration/
├── config/handeye_config.yaml     — defaults (mode, frame names, paths, method)
├── launch/handeye_capture.launch.py
├── handeye_calibration/
│   ├── sample_collector.py        — interactive capture node
│   ├── compute_calibration.py     — cv2.calibrateHandEye solver
│   └── publish_calibration.py     — static TF broadcaster of the result
└── README.md
```
