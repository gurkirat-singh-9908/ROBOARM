import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import serial
import time


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # Open serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Allow Arduino reset
            self.get_logger().info("Serial connected on /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        self.get_logger().info("Bridge listening to /joint_states ...")

        self.last_sent = None  # Prevent flooding


    def listener_callback(self, msg):

        if not msg.position or self.ser is None:
            return

        # Convert radians → servo degrees
        degrees = [int(math.degrees(p) + 90) for p in msg.position]
        degrees = [max(0, min(180, d)) for d in degrees]

        g = 0  # gripper placeholder

        command_values = degrees + [g]
        checksum = sum(command_values)  # safer checksum

        command_values.append(checksum)

        command = " ".join(str(v) for v in command_values) + "\n"

        # Avoid sending identical data repeatedly
        if command != self.last_sent:
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent: {command.strip()}")
            self.last_sent = command


def main():
    rclpy.init()
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
