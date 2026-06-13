"""
pick.py  —  object-agnostic pick task

Consumes the location element's contract and drives the arm through a pick.
Nothing is tomato-specific: the object is just a label (``object`` param) used
for logging; the detector/threshold logic lives in the location element. The
task is gated on detection — it does nothing until the object is seen
(``/location/detected``) at confidence ≥ ``pick_threshold`` for
``stable_frames`` consecutive ticks.

Two positioning modes:

  open_loop (default) — this node is the sole writer of /joint_states. It does
      a 2-DOF pixel servo (base + shoulder) from /location/center, then open-
      loop joint-delta dip / lift. Works with just a camera, no IK, no depth.

  ik — the location → ik chain already drives the arm onto the 3D target
      (/target_pose → ik_mover → joints), so this node does NOT write joints.
      It only sequences the gripper and nudges /target_pose down (approach) and
      up (lift). Use with the aruco source + ik.launch.py.

Topic contract:
  subscribes  /location/center      geometry_msgs/Point   x,y px, z radius px
              /location/confidence  std_msgs/Float64      0..1
              /location/detected    std_msgs/Bool         source has a target
              /target_pose          geometry_msgs/PoseStamped  (ik mode: cache)
              /roboarm/estop        std_msgs/Bool         global kill switch
  publishes   /joint_states         sensor_msgs/JointState  (open_loop only)
              /roboarm/gripper      std_msgs/Float64        open % (100=open)
              /target_pose          geometry_msgs/PoseStamped  (ik mode only)

State machine (same for both modes):
  SEARCH → CENTERING → GRASP_OPEN → APPROACH → GRASP_CLOSE → LIFT → DONE
Target lost during SEARCH/CENTERING reverts to SEARCH.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState

# Must match arduino_bridge JOINT_ORDER (s1..s6).
JOINT_NAMES = [
    'Revolute 43',   # s1 base
    'Revolute 44',   # s2 shoulder
    'Revolute 45',   # s3 elbow
    'Revolute 46',   # s4 wrist-a
    'Revolute 47',   # s5 wrist-b
    'Revolute 48',   # s6 wrist-c
]

SEARCH, CENTERING, GRASP_OPEN, APPROACH, GRASP_CLOSE, LIFT, DONE = (
    'SEARCH', 'CENTERING', 'GRASP_OPEN', 'APPROACH', 'GRASP_CLOSE', 'LIFT', 'DONE')


class Pick(Node):

    def __init__(self):
        super().__init__('pick')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('object', 'tomato')          # label only
        self.declare_parameter('positioning', 'open_loop')  # open_loop | ik
        self.declare_parameter('pick_threshold', 0.75)
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('tick_rate', 10.0)

        self.declare_parameter('center_x', 320.0)
        self.declare_parameter('center_y', 240.0)
        self.declare_parameter('center_deadzone_px', 15.0)
        self.declare_parameter('center_gain', 0.0015)
        self.declare_parameter('center_max_step', 0.02)

        # open_loop grasp motion (radians)
        self.declare_parameter('approach_shoulder_delta', 0.25)
        self.declare_parameter('approach_elbow_delta', 0.20)
        self.declare_parameter('lift_shoulder_delta', 0.40)
        # ik grasp motion (metres along z of the cached target)
        self.declare_parameter('approach_drop_z', -0.05)
        self.declare_parameter('lift_z', 0.10)
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('gripper_open_pct', 100.0)
        self.declare_parameter('gripper_close_pct', 0.0)
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('grip_time', 16.0)

        self._object = self.get_parameter('object').value
        self._mode = self.get_parameter('positioning').value
        self._threshold = float(self.get_parameter('pick_threshold').value)
        self._stable_need = int(self.get_parameter('stable_frames').value)
        self._timeout = float(self.get_parameter('detection_timeout').value)
        rate = float(self.get_parameter('tick_rate').value)
        self._cx_ref = float(self.get_parameter('center_x').value)
        self._cy_ref = float(self.get_parameter('center_y').value)
        self._deadzone = float(self.get_parameter('center_deadzone_px').value)
        self._gain = float(self.get_parameter('center_gain').value)
        self._max_step = float(self.get_parameter('center_max_step').value)
        self._appr_sh = float(self.get_parameter('approach_shoulder_delta').value)
        self._appr_el = float(self.get_parameter('approach_elbow_delta').value)
        self._lift_sh = float(self.get_parameter('lift_shoulder_delta').value)
        self._drop_z = float(self.get_parameter('approach_drop_z').value)
        self._lift_z = float(self.get_parameter('lift_z').value)
        self._base_frame = self.get_parameter('base_frame').value
        self._g_open = float(self.get_parameter('gripper_open_pct').value)
        self._g_close = float(self.get_parameter('gripper_close_pct').value)
        self._settle = float(self.get_parameter('settle_time').value)
        self._grip_time = float(self.get_parameter('grip_time').value)

        self._ik = (self._mode == 'ik')

        # ── IO ──────────────────────────────────────────────────────────────
        self.grip_pub = self.create_publisher(Float64, '/roboarm/gripper', 10)
        if self._ik:
            self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 1)
            self.create_subscription(PoseStamped, '/target_pose', self._on_target, 1)
        else:
            self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Point, '/location/center', self._on_center, 10)
        self.create_subscription(Float64, '/location/confidence', self._on_conf, 10)
        self.create_subscription(Bool, '/location/detected', self._on_detected, 10)
        self.create_subscription(Bool, '/roboarm/estop', self._on_estop, 10)

        # ── State ───────────────────────────────────────────────────────────
        self.positions = [0.0] * 6
        self._conf = 0.0
        self._detected = False
        self._center = None
        self._last_seen = 0.0
        self._last_target = None       # cached /target_pose (ik mode)
        self._stable = 0
        self._state = SEARCH
        self._state_t0 = time.time()
        self._approached = False
        self._estopped = False

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"pick ready — object='{self._object}', mode={self._mode}, "
            f"pick_threshold={self._threshold:.2f}.")

    # ── Subscriptions ────────────────────────────────────────────────────────

    def _on_center(self, msg: Point):
        self._center = (msg.x, msg.y, msg.z)
        self._last_seen = time.time()

    def _on_conf(self, msg: Float64):
        self._conf = float(msg.data)

    def _on_detected(self, msg: Bool):
        self._detected = bool(msg.data)
        if self._detected:
            self._last_seen = time.time()

    def _on_target(self, msg: PoseStamped):
        self._last_target = msg

    def _on_estop(self, msg: Bool):
        engaged = bool(msg.data)
        if engaged == self._estopped:
            return
        self._estopped = engaged
        if engaged:
            self.get_logger().warn('E-STOP ENGAGED — halting pick.')
        else:
            self._stable = 0
            self._approached = False
            self._enter(SEARCH)
            self.get_logger().warn('E-STOP released — restarting from SEARCH.')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _target_visible(self):
        return (time.time() - self._last_seen) <= self._timeout

    def _confident(self):
        return (self._target_visible() and self._detected
                and self._conf >= self._threshold)

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self.positions)
        self.joint_pub.publish(msg)

    def _publish_target_offset(self, dz: float):
        """ik mode: republish the cached target with z shifted by dz (metres)."""
        if self._last_target is None:
            self.get_logger().warn(
                'ik mode: no /target_pose cached yet — cannot offset.',
                throttle_duration_sec=2.0)
            return
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._last_target.header.frame_id or self._base_frame
        out.pose = self._last_target.pose
        out.pose.position.z += dz
        self.target_pub.publish(out)

    def _set_gripper(self, pct: float):
        m = Float64(); m.data = float(pct)
        self.grip_pub.publish(m)

    def _enter(self, state: str):
        self.get_logger().info(f'{self._state} → {state}')
        self._state = state
        self._state_t0 = time.time()

    def _elapsed(self):
        return time.time() - self._state_t0

    # ── FSM tick ──────────────────────────────────────────────────────────────

    def _tick(self):
        if self._estopped:
            return   # frozen — publish nothing, hold last commanded pose

        st = self._state

        if st == SEARCH:
            self._stable = self._stable + 1 if self._confident() else 0
            if self._stable >= self._stable_need:
                self.get_logger().info(
                    f"{self._object} locked (conf={self._conf:.2f}) — centering.")
                self._stable = 0
                self._enter(CENTERING)

        elif st == CENTERING:
            if not self._confident():
                self.get_logger().warn('Target lost while centering — back to search.')
                self._stable = 0
                self._enter(SEARCH)
                return
            if self._ik:
                # The location → ik chain is already driving the arm onto the
                # 3D target. Just confirm the lock has settled.
                self._stable += 1
                if self._stable >= self._stable_need:
                    self._stable = 0
                    self._enter(GRASP_OPEN)
                return
            cx, cy, _ = self._center
            ex = cx - self._cx_ref
            ey = cy - self._cy_ref
            if abs(ex) < self._deadzone and abs(ey) < self._deadzone:
                self._stable += 1
                if self._stable >= self._stable_need:
                    self._stable = 0
                    self._enter(GRASP_OPEN)
            else:
                self._stable = 0
                self._nudge(0, -ex)   # base follows horizontal error
                self._nudge(1, ey)    # shoulder follows vertical error
                self._publish_joints()

        elif st == GRASP_OPEN:
            self._set_gripper(self._g_open)
            if self._elapsed() >= self._grip_time:
                self._approached = False
                self._enter(APPROACH)

        elif st == APPROACH:
            if not self._approached:
                if self._ik:
                    self._publish_target_offset(self._drop_z)   # lower onto object
                else:
                    self.positions[1] += self._appr_sh          # dip shoulder
                    self.positions[2] += self._appr_el          # bend elbow
                    self._publish_joints()
                self._approached = True
            if self._elapsed() >= self._settle:
                self._enter(GRASP_CLOSE)

        elif st == GRASP_CLOSE:
            self._set_gripper(self._g_close)
            if self._elapsed() >= self._grip_time:
                self._enter(LIFT)

        elif st == LIFT:
            if self._elapsed() < 0.05:           # apply once on entry
                if self._ik:
                    self._publish_target_offset(self._lift_z)
                else:
                    self.positions[1] -= self._lift_sh
                    self._publish_joints()
            if self._elapsed() >= self._settle:
                self._enter(DONE)

        elif st == DONE:
            if self._elapsed() < 0.2:
                self.get_logger().info(f'{self._object} picked. Holding position.')

    def _nudge(self, idx: int, error: float):
        step = max(-self._max_step, min(self._max_step, error * self._gain))
        self.positions[idx] += step


def main():
    rclpy.init()
    node = Pick()
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
