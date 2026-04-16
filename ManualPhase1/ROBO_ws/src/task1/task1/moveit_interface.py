import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from rclpy.action import ActionClient


class MoveRobot(Node):

    def __init__(self):

        super().__init__('move_robot')

        self.client = ActionClient(self, MoveGroup, 'move_action')

        self.sub = self.create_subscription(
            Pose,
            '/target_pose',
            self.callback,
            10)

        self.get_logger().info("Waiting for MoveIt action server...")
        self.client.wait_for_server()

    def callback(self, msg):

        goal_msg = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = "arm"

        constraint = Constraints()

        # Example joint targets
        joint_names = [
            "Revolute 43",
            "Revolute 44",
            "Revolute 45",
            "Revolute 46",
            "Revolute 47",
            "Revolute 48"
        ]

        target_positions = [0.0, -0.3, 0.2, 0.0, 0.0, 0.0]

        for name, pos in zip(joint_names, target_positions):

            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0

            constraint.joint_constraints.append(jc)

        request.goal_constraints.append(constraint)

        goal_msg.request = request

        self.client.send_goal_async(goal_msg)


def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
