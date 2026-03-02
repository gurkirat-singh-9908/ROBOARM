import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
#import serial
import math

class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.listener_callback,
            10)

        #self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        #self.get_logger().info("Arduino Bridge Started")

    def listener_callback(self, msg):

        if len(msg.points) == 0:
            return

        # Take final trajectory point
        positions = msg.points[-1].positions

        # Convert radians → degrees
        degrees = [int(math.degrees(p)) for p in positions]

        # Clamp 0–180
        degrees = [max(0, min(180, d)) for d in degrees]

        # Send as comma-separated
        command = ",".join(map(str, degrees)) + "\n"
        #self.ser.write(command.encode())

        self.get_logger().info(f"Sent: {command.strip()}")

def main():
    rclpy.init()
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
