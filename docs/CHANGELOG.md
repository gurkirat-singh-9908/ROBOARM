# ROBOARM – Change Log / Commands

## Session: 2026-04-17 (automated scheduled task)

### Objective
Improve the ROS ↔ Arduino data-transfer link.

The pipeline is:
```
JointPublisher / JointTracker
        │  publishes  /joint_states  (sensor_msgs/JointState, radians)
        ▼
  ArduinoBridge (bridge_node.py)
        │  serial UART  "j0 j1 j2 j3 j4 j5 grip checksum\n"
        ▼
  Arduino (rec/rec.ino)
        │  drives servos via writeMicroseconds()
        ▼
  Physical robot arm (6 DOF + gripper)
```

---

### Bugs found & fixed

| # | File | Bug | Fix |
|---|------|-----|-----|
| 1 | `bridge_node.py` | Serial port `/dev/ttyUSB0` hardcoded | Made into ROS2 parameter `serial_port` |
| 2 | `bridge_node.py` | Baud rate 115200 hardcoded, mismatched with Arduino | Made into ROS2 parameter `baud_rate` (default 115200) |
| 3 | `bridge_node.py` | No reconnection if serial drops mid-run | Added 3-second watchdog timer that reconnects |
| 4 | `bridge_node.py` | No rate limiting beyond "last_sent" dedup; could flood serial on bursty topics | Added `min_send_interval` parameter (default 50 ms / 20 Hz cap) |
| 5 | `bridge_node.py` | No feedback read from Arduino | Added non-blocking `ser.read()` after each write; logs Arduino ACK/ERR lines |
| 6 | `rec/rec.ino` | **Critical**: baud rate was 9600, bridge sends at 115200 → all data garbled | Changed to `Serial.begin(115200)` |
| 7 | `rec/rec.ino` | **Critical**: only parsed 6 values with `sscanf("%d %d %d %d %d %d")` but bridge sends 8 (6 joints + gripper + checksum) → gripper & checksum silently dropped | Updated `sscanf` to read all 8 values |
| 8 | `rec/rec.ino` | Checksum sent by bridge was never validated on Arduino | Added checksum verification; mismatched commands are rejected with `ERR:checksum_fail` |
| 9 | `rec/rec.ino` | Used `servo.write(degrees)` which uses generic 544–2400 µs range | Replaced with `servo.writeMicroseconds()` using calibrated ranges from `read.me.txt` |
| 10 | `reciver.ino` | Baud rate was 9600 (mismatched with bridge) | Changed to `Serial.begin(115200)` |

---

### Files changed

```
ManualPhase1/ROBO_ws/src/arduino_bridge/arduino_bridge/bridge_node.py
Contrl/rec/rec.ino
Contrl/reciver.ino
```

---

### bridge_node.py – new ROS2 parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial device path (e.g. `/dev/ttyACM0` or `COM3` on Windows) |
| `baud_rate` | `115200` | Must match Arduino sketch |
| `min_send_interval` | `0.05` | Minimum seconds between serial writes (rate-limiter) |

**Override at launch:**
```bash
ros2 run arduino_bridge bridge_node --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p baud_rate:=115200 \
    -p min_send_interval:=0.05
```

---

### Serial protocol (unchanged format, now fully implemented end-to-end)

```
TX (bridge → Arduino):   "j0 j1 j2 j3 j4 j5 gripper checksum\n"
RX (Arduino → bridge):   "OK:j0,j1,j2,j3,j4,j5 grip=<g>"   on success
                          "ERR:checksum_fail(rx=X,calc=Y)"    on bad checksum
                          "ERR:bad_parse(got N)"              on parse error
```

All values are integers. `checksum = (j0+j1+j2+j3+j4+j5+gripper) & 0xFFFF`.

---

### Servo calibration applied (from read.me.txt)

| Servo | Pin | Min µs | Max µs |
|-------|-----|--------|--------|
| s1 (35 kg)  | 2 | 500 | 2400 |
| s2 (150 kg) | 3 | 500 | 2300 |
| s3 (150 kg) | 4 | 725 | 2050 |
| s4 (35 kg)  | 5 | 500 | 2400 |
| s5 (15 kg)  | 6 | 575 | 1900 |
| s6 (15 kg)  | 7 | 575 | 1900 |

---

### Known remaining issues / future work

1. **Joint 4 & 5 correction** in `bridge_node.py` negates the angle (`-degrees[4]`, `-degrees[5]`). After the 0–180 clamp these joints will be stuck at 0° whenever ROS publishes positive angles. Consider changing to `90 - degrees[4]` (mirror about 90°) once physical wiring is verified.

2. **Gripper topic** not yet connected. The gripper value is hardcoded to 0 in `bridge_node.py`. Wire up a `/gripper_command` subscriber when a gripper is added.

3. **Windows COM port**: On Windows the serial port will be `COM3`, `COM4`, etc. Override `serial_port` parameter accordingly.

4. **`contrl.ino`** in `Contrl/` folder is incomplete (setup/loop empty). Needs implementation when manual-phase hardware control is added.
