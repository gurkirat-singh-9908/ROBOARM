import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StartupState(Node):

    def __init__(self):
        super().__init__('startup_state')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(0.1, self.publish_state)

        self.msg = JointState()

        self.msg.name = [
            "Revolute_22",
            "Revolute_23",
            "Revolute_24",
            "Revolute_25",
            "Revolute_26",
            "Revolute_27"
        ]

        # Starting angles in radians
        self.msg.position = [
            1.57,
            -0.78,
            0.52,
            0.0,
            0.0,
            0.0
        ]


    def publish_state(self):

        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.pub.publish(self.msg)


def main():

    rclpy.init()

    node = StartupState()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
