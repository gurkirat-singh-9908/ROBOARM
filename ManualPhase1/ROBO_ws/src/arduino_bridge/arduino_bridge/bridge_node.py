import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import serial
import serial.tools.list_ports
import time


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # ── ROS2 parameters (override at launch or CLI) ──────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('min_send_interval', 0.05)   # seconds (20 Hz cap)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._min_interval = self.get_parameter('min_send_interval').value

        self.ser = None
        self._connect()

        # Reconnect watchdog – tries every 3 s if serial is gone
        self.create_timer(3.0, self._watchdog)

        # ── Subscribe to joint states ────────────────────────────────────────
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        self.get_logger().info(
            f"ArduinoBridge started  port={self._port}  baud={self._baud}")

        self._last_sent = None          # dedup: avoid resending identical commands
        self._last_send_time = 0.0     # rate-limit: cap at min_send_interval

    # ── Serial helpers ───────────────────────────────────────────────────────

    def _connect(self):
        """Attempt to open the serial port. Logs success/failure."""
        try:
            self.ser = serial.Serial(self._port, self._baud, timeout=1)
            time.sleep(2)             # let Arduino reset after DTR toggle
            self.get_logger().info(
                f"Serial connected on {self._port} at {self._baud} baud")
        except Exception as exc:
            self.get_logger().error(
                f"Serial connection failed ({self._port}): {exc}")
            self._log_available_ports()
            self.ser = None

    def _watchdog(self):
        """Reconnect if serial port dropped or was never opened."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial not open – attempting reconnect …")
            self._connect()

    def _log_available_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self.get_logger().info(f"Available serial ports: {ports}")
        else:
            self.get_logger().warn("No serial ports detected on this system.")

    # ── Joint-state callback ─────────────────────────────────────────────────

    def listener_callback(self, msg):
        if not msg.position:
            return
        if self.ser is None or not self.ser.is_open:
            return

        # ── Rate limit ────────────────────────────────────────────────────────
        now = time.time()
        if now - self._last_send_time < self._min_interval:
            return
        self._last_send_time = now

        # ── Convert radians → integer servo degrees ───────────────────────────
        degrees = [int(math.degrees(p)) for p in msg.position]

        if len(degrees) < 6:
            self.get_logger().warn(
                f"Expected ≥6 joint values, got {len(degrees)} – skipping.")
            return

        # ── Joint corrections (mechanical offsets for this robot) ─────────────
        # NOTE: joints 4 & 5 are negated; clamped to [0,180] below.
        #       If servos for those joints seem stuck at 0°, change these
        #       corrections to e.g.  90 - degrees[4]  to keep them in range.
        degrees[0] = 180 - degrees[0]
        degrees[1] = 180 + degrees[1]
        degrees[2] = 180 + degrees[2]
        degrees[3] = 180 + degrees[3]
        degrees[4] = -degrees[4]
        degrees[5] = -degrees[5]

        # Clamp to valid servo range
        degrees = [max(0, min(180, d)) for d in degrees]

        gripper = 0  # placeholder – extend when gripper topic is available

        payload = degrees[:6] + [gripper]   # 7 values

        # ── Checksum (simple sum) ─────────────────────────────────────────────
        checksum = sum(payload) & 0xFFFF    # 16-bit wrap-around

        # Format: "d0 d1 d2 d3 d4 d5 gripper checksum\n"
        command = " ".join(str(v) for v in payload) + f" {checksum}\n"

        # ── Dedup: only send if data changed ─────────────────────────────────
        if command == self._last_sent:
            return

        try:
            self.ser.write(command.encode('ascii'))
            self._last_sent = command
            self.get_logger().info(f"TX → {command.strip()}")

            # ── Non-blocking read: print any feedback line the Arduino echoes ─
            if self.ser.in_waiting:
                feedback = self.ser.readline().decode('ascii', errors='replace').strip()
                if feedback:
                    self.get_logger().info(f"RX ← {feedback}")

        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None     # watchdog will reconnect on next tick


def main():
    rclpy.init()
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
