# SliderBasedIKV1 – Slider-Based IK Control (Version 1)

First working implementation of manual arm control using desktop sliders.

## Structure

```
SliderBasedIKV1/
├── contrl/             # IK solvers (older snapshots; active versions are in Contrl/)
│   ├── main.py         # Analytical IK solver
│   ├── dh.py           # DH forward kinematics
│   ├── robot_visuals.py
│   ├── rotation_matrix.py
│   └── ...
├── gui/
│   ├── gui.ino          # Arduino sketch that receives slider values
│   ├── gui.py           # Python slider GUI (tkinter / pygame)
│   ├── updateServos.cpp # C++ helper for servo updates
│   └── updateServos.h
└── misc/
    ├── controller.py    # Python serial controller
    ├── reciver.ino      # Arduino receiver sketch
    └── sliders.py       # Alternative slider script
```

## How it works
1. `gui.py` presents sliders for each joint angle.
2. On change, it sends the angles over USB serial.
3. `gui.ino` / `reciver.ino` on the Arduino reads the serial data and calls `updateServos`.

## Note
This folder is a **historical snapshot**.  The active control scripts and the improved `rec/rec.ino` firmware (with checksum validation and `writeMicroseconds`) are in `Contrl/` at the repository root.
