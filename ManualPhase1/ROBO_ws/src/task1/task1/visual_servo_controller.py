import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose


class VisualServo(Node):

    def __init__(self):
        super().__init__('visual_servo')

        self.sub = self.create_subscription(
            Point,
            "/object_pixel",
            self.callback,
            10)

        self.pub = self.create_publisher(Pose, "/target_pose", 10)

        self.cx_ref = 320
        self.cy_ref = 240

        self.scale = 0.0007
        self.deadzone = 10

    def callback(self, msg):
        error_x = msg.x - self.cx_ref
        error_y = msg.y - self.cy_ref

        if abs(error_x) < self.deadzone and abs(error_y) < self.deadzone:
            return

        pose = Pose()
        pose.position.x = -error_x * self.scale
        pose.position.z = -error_y * self.scale
        self.pub.publish(pose)


def main():
    rclpy.init()
    node = VisualServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
