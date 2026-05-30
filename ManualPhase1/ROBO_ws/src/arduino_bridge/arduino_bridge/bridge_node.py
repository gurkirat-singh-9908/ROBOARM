"""
bridge_node.py  —  ROS2 → Arduino serial bridge

Subscribes to:
  /joint_states       sensor_msgs/JointState   arm angles (radians) from MoveIt2
  /roboarm/gripper    std_msgs/Float64          gripper open % [0-100]

Converts joint angles using robot-specific mechanical offsets and sends a
space-delimited command with checksum over serial to the Arduino at up to
`min_send_interval` Hz.

Data-consistency checks performed on every message:
  • all 6 named joints must be present
  • any joint changing more than `jump_threshold_deg` per update triggers a warning
  • values are clamped to [0, 180] before transmission
"""

import math
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

        self._port      = self.get_parameter('serial_port').value
        self._baud      = self.get_parameter('baud_rate').value
        self._interval  = self.get_parameter('min_send_interval').value
        self._jump_thr  = self.get_parameter('jump_threshold_deg').value

        # ── Serial ─────────────────────────────────────────────────────────────
        self.ser = None
        self._sim_mode = False
        self._connect_with_retries()
        self.create_timer(3.0, self._watchdog)

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states',     self._on_joints,  10)
        self.create_subscription(Float64,    '/roboarm/gripper',  self._on_gripper, 10)

        # ── State ──────────────────────────────────────────────────────────────
        self._gripper        = 0            # latest gripper % [0-100]
        self._last_degrees   = None         # for jump detection
        self._last_sent      = None         # dedup
        self._last_send_time = 0.0

        self.get_logger().info(
            f'ArduinoBridge ready  port={self._port}  baud={self._baud}  '
            f'jump_threshold={self._jump_thr}°')

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
        self._gripper = max(0.0, min(100.0, msg.data))

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
        gripper  = max(0, min(100, int(self._gripper)))
        payload  = degrees + [gripper]                  # 7 values
        checksum = sum(payload) & 0xFFFF
        command  = ' '.join(str(v) for v in payload) + f' {checksum}\n'

        if command == self._last_sent:
            return
        self._last_sent = command

        # ── Simulation mode: just log ──────────────────────────────────────────
        if self._sim_mode:
            self.get_logger().info(f'[SIM] → {command.strip()}')
            return

        if self.ser is None or not self.ser.is_open:
            return

        # ── Send to Arduino ────────────────────────────────────────────────────
        try:
            self.ser.write(command.encode('ascii'))
            self.get_logger().info(f'TX → {command.strip()}')

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
