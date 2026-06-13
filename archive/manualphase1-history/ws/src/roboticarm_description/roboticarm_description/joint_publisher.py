import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(0.05, self.publish_joints)

        self.start_time = time.time()

        self.joint_names = [
            "Revolute 43",
            "Revolute 44",
            "Revolute 45",
            "Revolute 46",
            "Revolute 47",
            "Revolute 48"
        ]

        self.min_angle = -1.0472
        self.max_angle = -0.523598776


    def publish_joints(self):

        t = time.time() - self.start_time

        # smooth oscillation between limits
        move = (math.sin(t) + 1) / 2
        angle = self.min_angle + move * (self.max_angle - self.min_angle)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.joint_names

        msg.position = [
            angle,          # Revolute 43 (moving)
            -1.5708,        # Revolute 44 (fixed)
            angle,          # Revolute 45 (moving)
            -1.5708,            # Revolute 46 (fixed)
            -1.5708,            # Revolute 47 (fixed)
            -1.5708             # Revolute 48 (fixed)
        ]

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = JointPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

