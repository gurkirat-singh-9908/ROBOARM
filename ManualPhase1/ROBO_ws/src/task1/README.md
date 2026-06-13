# task1 — Tomato Pick (Task 1)

High-level task nodes for the arm. Task 1 is **"pick a tomato"**, gated on a
detector confidence of **≥ 0.75**.

## Where it runs (PC / Pi split)

Detection + planning run on the **PC** (has a display). The
`arduino_bridge` runs on the **headless Pi** and talks to the Arduino.
They share one ROS graph (same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`), so
`/joint_states` and `/roboarm/gripper` published on the PC reach the Pi.
The `pick_tomato.launch.py` here is **PC-only** — it does *not* start the
bridge. (Endgame: everything moves onto the Pi; nothing about the topic
contract changes when it does.)

```
   PC (display)                                     Pi (headless)
┌──────────────────────────────────────┐        ┌─────────────────────────┐
│ tomato_detector ─/tomato/center────▶  │  ROS   │ arduino_bridge          │
│      │          /tomato/confidence    │  DDS   │   bridge_node ─▶ Arduino│
│   camera             │                │ ─────▶ │   /joint_states         │ ─▶ servos
│              pick_tomato (FSM) ────────────────▶│   /roboarm/gripper      │ ─▶ gripper
└──────────────────────────────────────┘        └─────────────────────────┘
```

- **`tomato_detector`** — HSV red segmentation (red wraps hue → two bands),
  largest blob, scores a confidence `0..1` from circularity × fill. Round,
  solid, well-filled red blobs score high; boxy / elongated / tiny blobs
  score low. Publishes every frame so the picker can detect target loss.
  *(Classical CV — no neural net; no model weights in this repo. Swap for a
  YOLO node later; the topic contract stays the same.)*
- **`pick_tomato`** — state machine, **sole writer of `/joint_states`** while
  running. Nothing moves until confidence ≥ `pick_threshold` (0.75) is held
  stable. Then: center (base+shoulder) → open jaws → dip (open-loop) →
  close jaws → lift.

`SEARCH → CENTERING → GRASP_OPEN → APPROACH → GRASP_CLOSE → LIFT → DONE`

## Run

On the **Pi** (headless) — bridge to the Arduino, no detection:

```bash
ros2 run arduino_bridge bridge_node --ros-args -p serial_port:=/dev/ttyUSB0
```

On the **PC** — detection + pick (this launch starts only these two):

```bash
ros2 launch task1 pick_tomato.launch.py
ros2 launch task1 pick_tomato.launch.py camera_index:=2 pick_threshold:=0.8

# or separately
ros2 run task1 tomato_detector --ros-args -p camera_index:=0
ros2 run task1 pick_tomato     --ros-args -p pick_threshold:=0.75
```

Set `ROS_DOMAIN_ID` equal and `ROS_LOCALHOST_ONLY=0` on both machines so
the topics cross the network. Do **not** also run `joint_tracker` — it
would fight `pick_tomato` over `/joint_states`.

## Kill switch (e-stop)

`/roboarm/estop` (`std_msgs/Bool`) is a global stop watched by **both** the
bridge and the picker:

- **True = KILL** — bridge instantly freezes (drops all serial writes, so
  the servos hold their last position); `pick_tomato` halts its FSM.
- **False = START** — bridge resumes; `pick_tomato` restarts from `SEARCH`.

The bridge is the authoritative stop — it sits on the Pi next to the
hardware and keeps freezing even if the PC dies. From any machine on the
ROS graph:

```bash
ros2 run task1 estop stop     # kill everything now
ros2 run task1 estop start    # resume / restart the pick

# equivalent without the helper:
ros2 topic pub --once /roboarm/estop std_msgs/Bool "{data: true}"   # kill
ros2 topic pub --once /roboarm/estop std_msgs/Bool "{data: false}"  # start
```

Run the helper while the bridge/picker are already up. Note: the gripper is
a DC motor whose pulse the Arduino runs to completion — an in-flight grip
pulse finishes before the freeze fully bites; everything else stops
immediately.

## Key parameters

| Node | Param | Default | Notes |
|------|-------|---------|-------|
| tomato_detector | `min_area` | 600 | px² floor; below this confidence = 0 |
| tomato_detector | `lower1/upper1`, `lower2/upper2` | red bands | HSV; retune per lighting |
| pick_tomato | `pick_threshold` | 0.75 | confidence to commit to a pick |
| pick_tomato | `stable_frames` | 5 | consecutive confident ticks required |
| pick_tomato | `center_gain` / `center_deadzone_px` | 0.0015 / 15 | 2-DOF pixel servo |
| pick_tomato | `approach_shoulder_delta` / `approach_elbow_delta` | 0.25 / 0.20 rad | **open-loop dip — tune to rig** |
| pick_tomato | `lift_shoulder_delta` | 0.40 rad | lift after grasp |
| pick_tomato | `gripper_open_pct` / `gripper_close_pct` | 100 / 0 | `/roboarm/gripper` % |
| pick_tomato | `grip_time` | 16 s | DC-gripper sweep time (no feedback) |

## Limits

No depth sensor → centering is 2-DOF from pixel error, and the dip/lift are
**open-loop preset joint deltas**. Tune `approach_*` / `lift_*` on the
physical arm. The confidence score is validated on synthetic shapes
(round=0.94, square=0.71, sliver=0.14) but HSV bounds need calibrating to
the real camera + lighting.

## Other nodes (generic blob demo, not task 1)

`object_detector`, `visual_servo`, `joint_tracker`, `moveit_interface` —
earlier generic colour-blob servo experiments. `moveit_interface` currently
ignores `/target_pose` (hardcoded joint targets — stub).
