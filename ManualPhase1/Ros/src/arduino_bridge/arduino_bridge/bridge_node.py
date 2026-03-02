import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        self.get_logger().info("Bridge listening to /joint_states ...")

    def listener_callback(self, msg):

        if not msg.position:
            return
    
        degrees = [int(math.degrees(p) + 90) for p in msg.position]
        degrees = [max(0, min(180, d)) for d in degrees]

        g = 0  # gripper placeholder

        command_values = degrees + [g]
        checksum = sum(command_values)

        command_values.append(checksum)

        command = " ".join(str(v) for v in command_values) + "\n"

        self.get_logger().info(f"Sending: {command.strip()}")

        # self.ser.write(command.encode())   # Uncomment when ready


def main():
    rclpy.init()
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
