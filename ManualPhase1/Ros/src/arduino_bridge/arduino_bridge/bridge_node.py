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

        # Ensure we have positions
        if not msg.position:
            return

        # Convert radians → degrees
        degrees = [int(math.degrees(p)) for p in msg.position]

        # Clamp 0–180 (for hobby servos)
        degrees = [max(0, min(180, d)) for d in degrees]

        command = ",".join(map(str, degrees))

        self.get_logger().info(f"JointState: {command}")


def main():
    rclpy.init()
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
