import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


class JointTracker(Node):

    def __init__(self):

        super().__init__('joint_tracker')

        self.subscription = self.create_subscription(
            Point,
            '/object_pixel',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.joint_names = [
            "Revolute 43",
            "Revolute 44",
            "Revolute 45",
            "Revolute 46",
            "Revolute 47",
            "Revolute 48"
        ]

        self.positions = [0.0]*6

        # camera center
        self.cx_ref = 320
        self.cy_ref = 240

        # tuning
        self.gain = 0.002
        self.deadzone = 10
        self.max_step = 0.02

    def callback(self,msg):

        cx = msg.x
        cy = msg.y

        ex = cx - self.cx_ref
        ey = cy - self.cy_ref

        if abs(ex) < self.deadzone and abs(ey) < self.deadzone:
            return

        move_base = ex * self.gain
        move_shoulder = ey * self.gain

        move_base = max(-self.max_step, min(self.max_step, move_base))
        move_shoulder = max(-self.max_step, min(self.max_step, move_shoulder))

        self.positions[0] -= move_base
        self.positions[1] += move_shoulder

        msg_out = JointState()
        msg_out.name = self.joint_names
        msg_out.position = self.positions

        self.publisher.publish(msg_out)


def main():

    rclpy.init()

    node = JointTracker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
