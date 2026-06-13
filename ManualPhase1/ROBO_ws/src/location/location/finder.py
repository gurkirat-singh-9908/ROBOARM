"""
finder.py  —  workspace-sweep sub-element of the location pipeline

A *source* (color_source / aruco_source) only sees what is in the current
frame. The finder closes the "object not in frame" gap: while nothing is
detected it walks the camera through a list of look-around poses so the
object eventually enters view; the moment a source reports a detection it
stops and yields control.

Contract:
  subscribes  /location/detected   std_msgs/Bool       any source has a target
  publishes   /target_pose         geometry_msgs/PoseStamped   search hints

The finder emits the SAME topic the real target uses (/target_pose → ik
element), but only while detected == False, so it never fights the live
target: detection flips the switch and the finder goes quiet. Publishing
poses (not joints) keeps the IK element the single owner of /joint_states.

Search poses are a flat list [x,y,z, x,y,z, ...] in ``base_frame`` (metres);
the finder dwells ``dwell_time`` seconds at each, looping. Tune to your reach.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


# A coarse arc in front of the arm — sweep left→right at two heights.
DEFAULT_POSES = [
    0.25, -0.15, 0.25,
    0.30,  0.00, 0.25,
    0.25,  0.15, 0.25,
    0.25,  0.15, 0.15,
    0.30,  0.00, 0.15,
    0.25, -0.15, 0.15,
]


class Finder(Node):

    def __init__(self):
        super().__init__('finder')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('dwell_time', 2.0)      # s at each search pose
        self.declare_parameter('lost_grace', 1.0)      # s of no-detect before resuming
        self.declare_parameter('search_poses', DEFAULT_POSES)

        self._frame = self.get_parameter('base_frame').value
        self._dwell = float(self.get_parameter('dwell_time').value)
        self._grace = float(self.get_parameter('lost_grace').value)
        flat = list(self.get_parameter('search_poses').value)
        if len(flat) < 3 or len(flat) % 3 != 0:
            self.get_logger().warn('search_poses not a multiple of 3 — using defaults.')
            flat = DEFAULT_POSES
        self._poses = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self._pub = self.create_publisher(PoseStamped, '/target_pose', 1)
        self.create_subscription(Bool, '/location/detected', self._on_detected, 10)

        self._detected = False
        self._last_detect_true = 0.0     # wall time of last detected=True
        self._idx = 0
        self._t_pose = self.get_clock().now()

        period = max(0.1, self._dwell)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'finder up — {len(self._poses)} search poses, dwell {self._dwell}s, '
            f'sweeping while /location/detected is false.')

    def _on_detected(self, msg: Bool):
        self._detected = bool(msg.data)
        if self._detected:
            self._last_detect_true = self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # Stay quiet while a target is live (or within the grace window after).
        if self._detected or (now - self._last_detect_true) < self._grace:
            return

        x, y, z = self._poses[self._idx]
        self._idx = (self._idx + 1) % len(self._poses)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)
        self.get_logger().info(
            f'searching → ({x:+.2f}, {y:+.2f}, {z:+.2f})',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = Finder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
