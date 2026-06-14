# location — the location element

A general, swappable **source of object position** for the arm pipeline. It
knows nothing about any specific task; it answers *"where is the object and how
sure am I"* and hands a target to the IK element.

```
location  →  ik  →  bridge  →  arm
```

## Sub-elements

> The camera is **not** here — it lives in its own `camera` package
> (`ros2 run camera camera`), publishing `/camera/image_raw`, because many
> elements need the feed. Every source below *subscribes* to that topic; none
> opens the device itself (except the standalone `tune` tool).

| Node | Role | Output |
|------|------|--------|
| `aruco_source` | ArUco marker detect — **metric 3D pose** via solvePnP | `/aruco/pose`, `/aruco/pixel_center`, `/aruco/id` |
| `color_source` | HSV colour-threshold detector (data-driven, per object) | `/location/center`, `/location/confidence`, `/location/detected` |
| `to_target` | reframe a source pose to `base_link` (+ hover) | `/target_pose` |
| `finder` | sweep the workspace while nothing is detected | `/target_pose` (search hints) |
| `tune` | interactive HSV tuner — writes `objects/<name>.yaml` | (saves config) |

A *source* is pluggable: `aruco_source` (3D today), `color_source` (pixel), or a
YOLO node later — all keep the same downstream contract. The **finder** closes
the "object not in the current frame" gap by walking the camera through
look-around poses until a source reports a detection, then going quiet.

## Data-driven detection (no object hard-coded)

What an object looks like lives in `objects/<name>.yaml`, not in code:

```yaml
object: tomato
lower1: [0, 120, 70]      # HSV band 1 (red wraps the hue axis → two bands)
upper1: [10, 255, 255]
lower2: [170, 120, 70]
upper2: [180, 255, 255]
min_area: 600
blur_ksize: 5
pick_threshold: 0.75
real_diameter_m: 0.06
```

Lookup order (`object_config.py`): user dir `$ROBOARM_OBJECTS` (default
`~/.roboarm/objects`) → packaged defaults in `share/location/objects`. If
neither has the file, `color_source` tells you to run the tuner and idles
(it never crashes, and downstream just sees `detected=False`):

```bash
ros2 run location tune --ros-args -p object:=pepper   # drag bars, press s to save
```

Tuned values save to the **writable** user dir, so a rebuild never wipes them;
the shipped `tomato.yaml` is the read-only default so Task 1 works out of the box.

## Run

```bash
# ArUco 3D source + reframe to base_link (feeds the IK element)
ros2 launch location location.launch.py source:=aruco

# colour source for an object
ros2 launch location location.launch.py source:=color object:=tomato

# individual nodes (camera must be up first — it's a separate package)
ros2 run camera camera --ros-args -p source:=0
ros2 run location color_source --ros-args -p object:=tomato
ros2 run location finder
```

## Contract

```
/location/center      geometry_msgs/Point        x,y px centre, z radius px
/location/confidence  std_msgs/Float64           0..1
/location/detected    std_msgs/Bool              confidence >= pick_threshold
/aruco/pose           geometry_msgs/PoseStamped  metric 3D (camera frame)
/target_pose          geometry_msgs/PoseStamped  base_link — consumed by ik_mover
```

`/aruco/*` topic names are unchanged from the old `ArUcoMarkerRos` package so
`handeye_calibration` keeps working untouched.

## Depth note

`aruco_source` gives true metric 3D (the marker provides scale), so its
`/target_pose` drives real IK directly. `color_source` gives a **pixel** centre
only — no depth yet — so it feeds the open-loop pixel-servo pick path. Fusing a
colour pixel with the ArUco-derived workspace plane to get 3D colour targets is
the next-phase job.
