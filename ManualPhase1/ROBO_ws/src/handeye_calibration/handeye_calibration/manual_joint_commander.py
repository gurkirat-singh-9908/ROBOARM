"""Interactive manual joint commander.

Publishes /joint_states for the 6 arm joints from stdin input, so the
arduino_bridge subscriber drives the physical arm AND robot_state_publisher
re-computes the base_link → gripper_1 TF from the same authoritative source.

Input formats accepted on stdin:
    j1 j2 j3 j4 j5 j6              — six values in the active unit (default deg)
    deg j1 .. j6                   — switch to degrees for this line
    rad j1 .. j6                   — switch to radians for this line
    set <i> <v>                    — set joint i (1-6) to value <v>
    home                           — reset to all zeros
    show                           — print current command
    quit / q                       — exit

Continuous publishing at 20 Hz keeps consumers fed; a held command stays
constant until you type another line.
"""
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_ORDER = [
    'Revolute 43',
    'Revolute 44',
    'Revolute 45',
    'Revolute 46',
    'Revolute 47',
    'Revolute 48',
]


class ManualJointCommander(Node):

    def __init__(self):
        super().__init__('manual_joint_commander')

        self.declare_parameter('rate_hz',     20.0)
        self.declare_parameter('input_unit',  'deg')   # 'deg' or 'rad'
        self.declare_parameter('initial',     [0.0] * 6)

        rate_hz = float(self.get_parameter('rate_hz').value)
        unit_p  = str(self.get_parameter('input_unit').value).lower()
        if unit_p not in ('deg', 'rad'):
            self.get_logger().warn(f"input_unit={unit_p!r} invalid; using 'deg'")
            unit_p = 'deg'
        self._default_unit = unit_p

        init = list(self.get_parameter('initial').value)
        if len(init) != 6:
            self.get_logger().warn(
                f"`initial` length {len(init)} != 6; padding/truncating to zeros")
            init = (init + [0.0] * 6)[:6]

        # Internal state always stored in radians.
        self._angles_rad = [float(v) for v in init]
        self._lock = threading.Lock()

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / max(rate_hz, 1.0), self._publish)

        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        self.get_logger().info(
            f"Manual joint commander ready. Rate={rate_hz} Hz  unit={self._default_unit}")
        self._print_help()

    def _print_help(self):
        self.get_logger().info(
            "Commands: '<j1..j6>' | 'deg <j1..j6>' | 'rad <j1..j6>' | "
            "'set <i> <v>' | 'home' | 'show' | 'q'")

    def _publish(self):
        with self._lock:
            angles = list(self._angles_rad)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_ORDER
        msg.position = angles
        self._pub.publish(msg)

    def _convert_to_rad(self, vals, unit):
        if unit == 'deg':
            return [math.radians(v) for v in vals]
        return list(vals)

    def _format_angles(self):
        deg = [math.degrees(v) for v in self._angles_rad]
        return "  ".join(f"j{i+1}={d:7.2f}°" for i, d in enumerate(deg))

    def _stdin_loop(self):
        for raw in sys.stdin:
            line = raw.strip()
            if not line:
                continue
            tok = line.split()
            cmd = tok[0].lower()

            try:
                if cmd in ('q', 'quit', 'exit'):
                    self.get_logger().info("Quit requested.")
                    rclpy.shutdown()
                    return

                if cmd in ('h', 'help', '?'):
                    self._print_help()
                    continue

                if cmd == 'show':
                    self.get_logger().info(f"Current  {self._format_angles()}")
                    continue

                if cmd == 'home':
                    with self._lock:
                        self._angles_rad = [0.0] * 6
                    self.get_logger().info(f"Home     {self._format_angles()}")
                    continue

                if cmd == 'set':
                    if len(tok) != 3:
                        self.get_logger().warn("Usage: set <i 1-6> <value>")
                        continue
                    i = int(tok[1]) - 1
                    if not 0 <= i < 6:
                        self.get_logger().warn("Joint index must be 1-6")
                        continue
                    v = float(tok[2])
                    rad = self._convert_to_rad([v], self._default_unit)[0]
                    with self._lock:
                        self._angles_rad[i] = rad
                    self.get_logger().info(f"Set j{i+1}  {self._format_angles()}")
                    continue

                # Unit prefix?
                if cmd in ('deg', 'rad'):
                    unit = cmd
                    nums = tok[1:]
                else:
                    unit = self._default_unit
                    nums = tok

                if len(nums) != 6:
                    self.get_logger().warn(
                        f"Need 6 values, got {len(nums)}: {line!r}")
                    continue

                vals = [float(x) for x in nums]
                rad = self._convert_to_rad(vals, unit)
                with self._lock:
                    self._angles_rad = rad
                self.get_logger().info(f"Updated  {self._format_angles()}")

            except ValueError as exc:
                self.get_logger().warn(f"Parse error on {line!r}: {exc}")


def main():
    rclpy.init()
    node = ManualJointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
