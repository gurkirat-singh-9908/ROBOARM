import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from moveit_commander import MoveGroupCommander


class ObjectFollower(Node):

    def __init__(self):

        super().__init__('object_follower')

        self.group = MoveGroupCommander("arm")

        self.sub = self.create_subscription(
            Point,
            '/object_center',
            self.callback,
            10)

        pose = self.group.get_current_pose().pose

        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z

        self.scale = 0.0007

    def callback(self,msg):

        cx = msg.x
        cy = msg.y

        cx_ref = 320
        cy_ref = 240

        dx = (cx - cx_ref) * self.scale
        dz = (cy - cy_ref) * self.scale

        self.x -= dx
        self.z -= dz

        pose = self.group.get_current_pose().pose

        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z

        self.group.set_pose_target(pose)

        self.group.go(wait=False)


def main():

    rclpy.init()

    node = ObjectFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
