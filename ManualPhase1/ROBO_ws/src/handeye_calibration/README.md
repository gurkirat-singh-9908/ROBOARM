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

6. **Run the autonomous pipeline tail**

   Once calibration is broadcast, run the IK pipeline and the ArUco-to-target
   bridge in two terminals:

   ```bash
   # Terminal A — IK + Arduino bridge
   ros2 launch roboticarm_moveit2 ik_pipeline.launch.py

   # Terminal B — camera + aruco_detect + publish_calibration + transform
   ros2 launch handeye_calibration aruco_to_target.launch.py
   ```

   `aruco_to_target` subscribes to `/aruco/pose` (camera frame), looks up the
   static TF `base_link → camera_1`, transforms the marker pose into
   `base_link`, and republishes it on `/target_pose` for `ik_mover`.

   Useful overrides:

   ```bash
   ros2 launch handeye_calibration aruco_to_target.launch.py \
       hover_offset_z:=0.10 \
       target_marker_id:=4 \
       smooth_alpha:=0.3 \
       publish_rate:=10.0
   ```

## Frame naming — `camera_1` vs `camera_optical`

The URDF mounts a nominal `camera_1` link to `w_c_1` via the fixed joint
`Rigid 50`. That placement is approximate and exists only for visualization
/ sim. The hand-eye calibration publishes its result on a **different** frame
name — `camera_optical` — to avoid two TF publishers fighting over `camera_1`.

| Frame             | Source                          | Used for                |
|-------------------|----------------------------------|-------------------------|
| `camera_1`        | URDF `Rigid 50` (robot_state_publisher) | RViz mesh, sim         |
| `camera_optical`  | `publish_calibration` (static TF)       | Real-world ArUco poses |

`aruco_detect` stamps poses with `camera_frame` (default `camera_optical`),
so the autonomous pipeline transforms them through the calibration result.
Sim launch overrides this back to `camera_1` because `camera_optical` does
not exist without a calibration broadcast.

## Pipeline data flow

```
camera_node ──▶ /camera/image_raw
                     │
            aruco_detect  ──▶ /aruco/pose   (camera_1 frame)
                                  │
       publish_calibration: static TF base_link → camera_1
                                  │
            aruco_to_target ──▶ /target_pose (base_link frame, +hover_z)
                                  │
                            ik_mover (existing) ──▶ /arm_controller/joint_trajectory
                                                ──▶ Arduino bridge → servos
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
├── config/handeye_config.yaml          — defaults (mode, frame names, paths, method)
├── launch/
│   ├── handeye_capture.launch.py       — camera + aruco + sample_collector
│   ├── handeye_compute.launch.py       — solver + static TF broadcaster
│   ├── handeye_sim.launch.py           — software-only sim (no hardware)
│   └── aruco_to_target.launch.py       — autonomous tail to /target_pose
├── handeye_calibration/
│   ├── sample_collector.py             — interactive capture node
│   ├── compute_calibration.py          — cv2.calibrateHandEye solver
│   ├── publish_calibration.py          — static TF broadcaster of the result
│   ├── sim_aruco_publisher.py          — fake /aruco/pose from TF (sim mode)
│   └── aruco_to_target.py              — /aruco/pose → /target_pose (base frame)
└── README.md
```
