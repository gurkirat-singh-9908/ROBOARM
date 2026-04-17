# vission – Computer Vision Utilities

HSV colour-based object detection scripts for the robotic arm camera.

## Files

| File | Description |
|------|-------------|
| `initialblue.py` | Blue object detector with tunable HSV sliders (`H: 100-113`, `S: 118-230`, `V: 113-255`) |
| `tempdetection.py` | Generic colour detector with full-range HSV sliders for calibration |

## Usage

```bash
python initialblue.py    # blue object detection (camera index 0)
python tempdetection.py  # calibration tool (camera index 1)
```

Both scripts open a `Trackbars` window for live HSV tuning.
Use these scripts to calibrate colour ranges before running the full vision pipeline.

## Dependencies
- OpenCV (`cv2`)
- numpy
