"""
bridge_node.py  —  ROS2 → Arduino serial bridge

Subscribes to:
  /joint_states       sensor_msgs/JointState   arm angles (radians) from MoveIt2
  /roboarm/gripper    std_msgs/Float64          gripper open % [0-100]

Converts joint angles using robot-specific mechanical offsets and sends a
space-delimited command with checksum over serial to the Arduino at up to
`min_send_interval` Hz.

Gripper control model:
  The gripper is a DC motor with no position feedback. The host owns the
  last commanded percent, persisted to disk. On every new target percent
  we send a signed pulse-duration (ms) to the Arduino; sign = direction,
  magnitude = run-time. Field [6] of the serial packet carries this
  signed-ms value (replacing the old absolute-percent semantic).

Data-consistency checks performed on every message:
  • all 6 named joints must be present
  • any joint changing more than `jump_threshold_deg` per update triggers a warning
  • values are clamped to [0, 180] before transmission
"""

import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import serial
import serial.tools.list_ports


# Joint names in the order the Arduino expects them (s1 … s6)
JOINT_ORDER = [
    'Revolute 43',   # s1 – waist
    'Revolute 44',   # s2 – shoulder
    'Revolute 45',   # s3 – elbow
    'Revolute 46',   # s4 – wrist-a
    'Revolute 47',   # s5 – wrist-b
    'Revolute 48',   # s6 – wrist-c
]

# Default location for the gripper/joint state file. Lives inside the
# arduino_bridge source package so it travels with the workspace.
_DEFAULT_STATE_PATH = os.path.expanduser(
    '~/ROBOARM/ManualPhase1/ROBO_ws/src/arduino_bridge/gripper_state.txt'
)


def _joints_from_msg(msg: JointState):
    """
    Extract positions for JOINT_ORDER from a JointState message.
    Returns list[float] (radians) in s1..s6 order, or None if any joint missing.
    """
    name_to_pos = dict(zip(msg.name, msg.position))
    out = []
    for name in JOINT_ORDER:
        if name not in name_to_pos:
            return None
        out.append(name_to_pos[name])
    return out


def _radians_to_servo_degrees(rad_list: list[float]) -> list[int]:
    """
    Convert IK joint angles (radians) → servo PWM degrees (0-180).
    Applies per-joint mechanical offset corrections for this robot.
    """
    deg = [math.degrees(r) for r in rad_list]

    # Robot-specific mechanical offsets  (adjust if a servo is mirrored)
    deg[0] = 180 - deg[0]   # Revolute 43
    deg[1] = 180 + deg[1]   # Revolute 44
    deg[2] = 180 + deg[2]   # Revolute 45
    deg[3] = 180 + deg[3]   # Revolute 46
    deg[4] =      -deg[4]   # Revolute 47
    deg[5] =      -deg[5]   # Revolute 48

    return [max(0, min(180, int(d))) for d in deg]


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # ── ROS parameters ─────────────────────────────────────────────────────
        self.declare_parameter('serial_port',        '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',           115200)
        self.declare_parameter('min_send_interval',   0.05)   # 20 Hz cap
        self.declare_parameter('jump_threshold_deg',  25.0)   # warn threshold

        # Gripper (DC motor, no feedback) — host owns state.
        # ms_per_percent: calibrated 15.0 s full mechanical sweep, derated to
        # 13.5 s (1.5 s margin) so the jaws stop short of the hard stop and
        # apply grip traction instead of stalling. 13500 ms / 100 % = 135.0.
        # max_pulse_ms: 14000 covers a full 0→100 % command in a single pulse
        # with 500 ms slack, so the bridge doesn't have to chunk the sweep.
        self.declare_parameter('gripper_state_path',     _DEFAULT_STATE_PATH)
        self.declare_parameter('gripper_ms_per_percent', 150.0)
        self.declare_parameter('gripper_max_pulse_ms',   16000)

        self._port      = self.get_parameter('serial_port').value
        self._baud      = self.get_parameter('baud_rate').value
        self._interval  = self.get_parameter('min_send_interval').value
        self._jump_thr  = self.get_parameter('jump_threshold_deg').value

        self._state_path     = os.path.expanduser(
            self.get_parameter('gripper_state_path').value)
        self._ms_per_percent = float(self.get_parameter('gripper_ms_per_percent').value)
        self._max_pulse_ms   = int(self.get_parameter('gripper_max_pulse_ms').value)

        # ── Serial ─────────────────────────────────────────────────────────────
        self.ser = None
        self._sim_mode = False
        self._connect_with_retries()
        self.create_timer(3.0, self._watchdog)

        # ── State ──────────────────────────────────────────────────────────────
        self._last_degrees      = None   # most recent servo degrees, for save + jump detect
        self._last_sent         = None   # dedup
        self._last_send_time    = 0.0
        self._pending_gripper_ms = 0     # signed pulse to send on next packet

        self._last_gripper_pct, persisted_degrees = self._load_state()
        if persisted_degrees is not None:
            self._last_degrees = persisted_degrees

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states',     self._on_joints,  10)
        self.create_subscription(Float64,    '/roboarm/gripper',  self._on_gripper, 10)

        # ── Shutdown handlers so Ctrl-C / SIGTERM still persist state ──────────
        self._stopped = False
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self._signal_handler)
            except (ValueError, OSError):
                pass   # Not on main thread — rclpy will handle it.

        self.get_logger().info(
            f'ArduinoBridge ready  port={self._port}  baud={self._baud}  '
            f'jump_threshold={self._jump_thr}°  '
            f'gripper_pct={self._last_gripper_pct:.1f}  '
            f'ms/%={self._ms_per_percent}')

    # ── State file ─────────────────────────────────────────────────────────────

    def _load_state(self):
        """
        Returns (gripper_pct: float, degrees: list[int] | None).
        Missing or unreadable file → (0.0, None).
        """
        try:
            with open(self._state_path, 'r') as f:
                parts = f.read().split()
            if not parts:
                return 0.0, None
            gripper_pct = float(parts[0])
            degrees = None
            if len(parts) >= 7:
                degrees = [int(p) for p in parts[1:7]]
            self.get_logger().info(
                f'Loaded gripper state from {self._state_path}: '
                f'pct={gripper_pct:.1f} degrees={degrees}')
            return gripper_pct, degrees
        except FileNotFoundError:
            self.get_logger().info(
                f'No gripper state file at {self._state_path} — starting at 0%.')
            return 0.0, None
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to load gripper state ({exc}) — starting at 0%.')
            return 0.0, None

    def _save_state(self):
        """Atomic write of gripper % + last joint degrees to the state file."""
        try:
            os.makedirs(os.path.dirname(self._state_path), exist_ok=True)
            tmp_path = self._state_path + '.tmp'
            deg_part = (' ' + ' '.join(str(d) for d in self._last_degrees)
                        if self._last_degrees is not None else '')
            with open(tmp_path, 'w') as f:
                f.write(f'{self._last_gripper_pct:.3f}{deg_part}\n')
            os.replace(tmp_path, self._state_path)
        except Exception as exc:
            self.get_logger().warn(f'Failed to save gripper state: {exc}')

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def stop_communication(self):
        """Flush state to disk and close the serial port. Idempotent."""
        if self._stopped:
            return
        self._stopped = True
        self._save_state()
        self.get_logger().info(
            f'Saved gripper state: pct={self._last_gripper_pct:.1f} '
            f'degrees={self._last_degrees}')
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _signal_handler(self, signum, frame):
        self.get_logger().info(f'Received signal {signum} — shutting down.')
        self.stop_communication()
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        self.stop_communication()
        return super().destroy_node()

    # ── Serial helpers ─────────────────────────────────────────────────────────

    def _connect(self) -> bool:
        """Single connection attempt. Returns True on success."""
        try:
            self.ser = serial.Serial(self._port, self._baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'Serial connected  {self._port} @ {self._baud}')
            return True
        except Exception as exc:
            self.get_logger().warn(f'Serial open failed ({self._port}): {exc}')
            self.ser = None
            return False

    def _connect_with_retries(self, max_attempts: int = 3):
        """Try to connect max_attempts times. On total failure enter simulation mode."""
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(
                f'Connecting to Arduino on {self._port}  (attempt {attempt}/{max_attempts}) …')
            if self._connect():
                return
            if attempt < max_attempts:
                time.sleep(1)

        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.get_logger().warn(
            f'Could not connect after {max_attempts} attempts. '
            f'Available ports: {ports if ports else "none"}')
        self.get_logger().warn('*** Running in SIMULATION MODE — serial TX disabled ***')
        self._sim_mode = True

    def _watchdog(self):
        if self._sim_mode:
            return   # don't spam logs once we've given up on hardware
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial lost — attempting reconnect …')
            self._connect()

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _on_gripper(self, msg: Float64):
        """
        Convert a new target percent into a signed pulse-ms relative to the
        last commanded percent. Clamp pulse magnitude to the configured cap;
        advance the saved percent by only what was actually commanded so the
        next pulse picks up any leftover travel.
        """
        target = max(0.0, min(100.0, float(msg.data)))
        delta_pct = target - self._last_gripper_pct
        raw_ms = int(round(delta_pct * self._ms_per_percent))
        pulse_ms = max(-self._max_pulse_ms, min(self._max_pulse_ms, raw_ms))

        if pulse_ms == 0:
            return

        # Advance our notion of position by what we actually sent.
        commanded_delta_pct = pulse_ms / self._ms_per_percent
        self._last_gripper_pct = max(
            0.0, min(100.0, self._last_gripper_pct + commanded_delta_pct))

        self._pending_gripper_ms = pulse_ms
        self._save_state()
        self.get_logger().info(
            f'Gripper target={target:.1f}%  pulse={pulse_ms} ms  '
            f'last_pct={self._last_gripper_pct:.1f}%')

    def _on_joints(self, msg: JointState):
        # ── Validate all joints present ────────────────────────────────────────
        radians = _joints_from_msg(msg)
        if radians is None:
            missing = [n for n in JOINT_ORDER if n not in msg.name]
            self.get_logger().warn(
                f'JointState missing joints {missing} — skipping.')
            return

        # ── Convert to servo degrees ───────────────────────────────────────────
        degrees = _radians_to_servo_degrees(radians)

        # ── Jump detection ─────────────────────────────────────────────────────
        if self._last_degrees is not None:
            for i, (prev, curr) in enumerate(zip(self._last_degrees, degrees)):
                delta = abs(curr - prev)
                if delta > self._jump_thr:
                    self.get_logger().warn(
                        f'Joint {JOINT_ORDER[i]} jumped {delta:.1f}° '
                        f'({prev}° → {curr}°) — possible discontinuity.')
        self._last_degrees = degrees

        # ── Rate limit ─────────────────────────────────────────────────────────
        now = time.time()
        if now - self._last_send_time < self._interval:
            return
        self._last_send_time = now

        # ── Build serial command ───────────────────────────────────────────────
        # Field [6] is signed gripper pulse-ms (consumed once, then cleared).
        gripper_ms = int(self._pending_gripper_ms)
        payload  = degrees + [gripper_ms]                # 7 values
        checksum = sum(payload) & 0xFFFF
        command  = ' '.join(str(v) for v in payload) + f' {checksum}\n'

        # Dedup only when no gripper pulse is pending — otherwise a repeated
        # arm pose with a fresh pulse would be silently dropped.
        if gripper_ms == 0 and command == self._last_sent:
            return
        self._last_sent = command

        # ── Simulation mode: just log ──────────────────────────────────────────
        if self._sim_mode:
            self.get_logger().info(f'[SIM] → {command.strip()}')
            self._pending_gripper_ms = 0
            return

        if self.ser is None or not self.ser.is_open:
            return

        # ── Send to Arduino ────────────────────────────────────────────────────
        try:
            self.ser.write(command.encode('ascii'))
            self.get_logger().info(f'TX → {command.strip()}')
            self._pending_gripper_ms = 0    # consume the pulse on successful TX

            if self.ser.in_waiting:
                fb = self.ser.readline().decode('ascii', errors='replace').strip()
                if fb:
                    self.get_logger().info(f'RX ← {fb}')

        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None     # watchdog reconnects on next tick


def main():
    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_communication()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
