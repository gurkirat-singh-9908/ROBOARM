# tasks — the task element

Object-agnostic high-level orchestrators that sit on top of the pipeline:

```
location  →  ik  →  bridge  →  arm
                ▲
              tasks   (sequence the job: search → centre → grasp → lift)
```

Nothing here is tomato-specific. The object is a label (`object:=` param); what
it looks like and how confident a detection must be lives in the **location**
element (`objects/<name>.yaml`). Task 1 ("pick a tomato") is just
`pick object:=tomato`.

## Nodes

| Node | Role |
|------|------|
| `pick` | gated pick state machine, two positioning modes |
| `estop` | CLI helper for the global kill switch |

## `pick`

Consumes the location contract and runs:

```
SEARCH → CENTERING → GRASP_OPEN → APPROACH → GRASP_CLOSE → LIFT → DONE
```

Nothing moves until the object is seen (`/location/detected`) at confidence ≥
`pick_threshold` (0.75) held stable for `stable_frames` ticks.

Two positioning modes (`positioning:=`):

- **open_loop** (default) — `pick` is the sole writer of `/joint_states`. 2-DOF
  pixel servo (base + shoulder) from `/location/center`, then open-loop joint-
  delta dip / lift. Camera only, no IK, no depth.
- **ik** — the `location → ik` chain already drives the arm onto the 3D target
  (`/target_pose → ik_mover → /joint_states`), so `pick` does **not** write
  joints; it only sequences the gripper and nudges `/target_pose` down
  (approach) / up (lift). Use with `aruco_source` + `ik.launch.py`.

## Run

PC side, colour + open-loop path (does **not** start the bridge — that runs on
the Pi):

```bash
ros2 launch tasks pick.launch.py                       # object:=tomato default
ros2 launch tasks pick.launch.py object:=tomato camera_index:=2 pick_threshold:=0.8
```

ArUco + real-IK path (three pieces):

```bash
ros2 launch location location.launch.py source:=aruco     # PC: detect → /target_pose
ros2 launch roboticarm_moveit2 ik.launch.py               # IK + interpolator + bridge
ros2 run tasks pick --ros-args -p positioning:=ik
```

Set `ROS_DOMAIN_ID` equal and `ROS_LOCALHOST_ONLY=0` on PC + Pi so topics cross
the network.

## Kill switch (e-stop)

`/roboarm/estop` (`std_msgs/Bool`) is a global stop watched by both the bridge
and `pick`:

```bash
ros2 run tasks estop stop      # KILL — bridge freezes, gripper brakes, pick halts
ros2 run tasks estop start     # resume — bridge resumes, pick restarts from SEARCH

# without the helper:
ros2 topic pub --once /roboarm/estop std_msgs/Bool "{data: true}"    # kill
ros2 topic pub --once /roboarm/estop std_msgs/Bool "{data: false}"   # start
```

On KILL the bridge also sends the Arduino an `X` control byte, braking the
gripper DC motor immediately — even mid-pulse. (`G` on release.)

## Key parameters

| Param | Default | Notes |
|-------|---------|-------|
| `object` | tomato | label + which `objects/<name>.yaml` the source uses |
| `positioning` | open_loop | `open_loop` \| `ik` |
| `pick_threshold` | 0.75 | confidence to commit |
| `stable_frames` | 5 | consecutive confident ticks |
| `center_gain` / `center_deadzone_px` | 0.0015 / 15 | open_loop pixel servo |
| `approach_shoulder_delta` / `approach_elbow_delta` | 0.25 / 0.20 rad | open_loop dip — tune to rig |
| `lift_shoulder_delta` | 0.40 rad | open_loop lift |
| `approach_drop_z` / `lift_z` | -0.05 / 0.10 m | ik-mode target offsets |
| `gripper_open_pct` / `gripper_close_pct` | 100 / 0 | `/roboarm/gripper` % |
| `grip_time` | 16 s | DC-gripper sweep (no feedback) |

## Limits

`open_loop` centering is 2-DOF from pixel error and the dip/lift are preset
joint deltas — tune `approach_*` / `lift_*` to the physical arm. `ik` mode needs
a metric `/target_pose` (the ArUco source provides one) and the IK stack running.
