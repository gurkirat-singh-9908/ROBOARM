"""
pick_tomato.py  —  Task 1 orchestrator: pick a tomato

State machine that consumes the tomato_detector output and drives the arm
through a pick. The whole task is gated on detector confidence: nothing
moves until a tomato is seen at >= `pick_threshold` (default 0.75) for
`stable_frames` consecutive ticks.

Topic contract:
  subscribes  /tomato/center      geometry_msgs/Point   x,y px centre, z radius
              /tomato/confidence  std_msgs/Float64      0..1
  publishes   /joint_states       sensor_msgs/JointState  6 joints, radians
              /roboarm/gripper    std_msgs/Float64        open % (100=open, 0=closed)

This node is the SOLE writer of /joint_states while the task runs — do not
also run joint_tracker, or the two will fight over the topic.

State machine:
  SEARCH      wait for confidence >= threshold, held stable        → CENTERING
  CENTERING   P-control base+shoulder to bring blob to image centre → GRASP_OPEN
  GRASP_OPEN  open the jaws, settle                                 → APPROACH
  APPROACH    open-loop dip toward the object (preset joint deltas) → GRASP_CLOSE
  GRASP_CLOSE close the jaws onto the tomato, settle                → LIFT
  LIFT        raise the arm with the tomato held                    → DONE
  DONE        log success, hold position
Target lost (confidence drops / stale) during SEARCH or CENTERING reverts
to SEARCH.

Assumptions / limits (no depth sensor on this arm):
  • Centering is 2-DOF (base + shoulder) from pixel error only.
  • APPROACH / LIFT are open-loop preset joint deltas — tune
    `approach_*` / `lift_shoulder_delta` to the physical setup.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
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


class PickTomato(Node):

    def __init__(self):
        super().__init__('pick_tomato')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('pick_threshold', 0.75)    # confidence to commit
        self.declare_parameter('stable_frames', 5)        # consecutive ticks
        self.declare_parameter('detection_timeout', 0.5)  # s before target is "stale"
        self.declare_parameter('tick_rate', 10.0)         # Hz FSM + joint publish

        self.declare_parameter('center_x', 320.0)         # image centre px
        self.declare_parameter('center_y', 240.0)
        self.declare_parameter('center_deadzone_px', 15.0)
        self.declare_parameter('center_gain', 0.0015)     # rad per px error
        self.declare_parameter('center_max_step', 0.02)   # rad per tick clamp

        # Open-loop grasp motion (radians). Tune to the rig.
        self.declare_parameter('approach_shoulder_delta', 0.25)
        self.declare_parameter('approach_elbow_delta', 0.20)
        self.declare_parameter('lift_shoulder_delta', 0.40)

        self.declare_parameter('gripper_open_pct', 100.0)
        self.declare_parameter('gripper_close_pct', 0.0)
        self.declare_parameter('settle_time', 1.0)        # s for arm moves
        self.declare_parameter('grip_time', 16.0)         # s for jaw sweep (DC motor)

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
        self._g_open = float(self.get_parameter('gripper_open_pct').value)
        self._g_close = float(self.get_parameter('gripper_close_pct').value)
        self._settle = float(self.get_parameter('settle_time').value)
        self._grip_time = float(self.get_parameter('grip_time').value)

        # ── IO ──────────────────────────────────────────────────────────────
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.grip_pub = self.create_publisher(Float64, '/roboarm/gripper', 10)
        self.create_subscription(Point, '/tomato/center', self._on_center, 10)
        self.create_subscription(Float64, '/tomato/confidence', self._on_conf, 10)
        # E-stop: True halts the FSM; False (re)starts it from SEARCH.
        self.create_subscription(Bool, '/roboarm/estop', self._on_estop, 10)

        # ── State ───────────────────────────────────────────────────────────
        self.positions = [0.0] * 6
        self._conf = 0.0
        self._center = None
        self._last_seen = 0.0
        self._stable = 0
        self._state = SEARCH
        self._state_t0 = time.time()
        self._approached = False
        self._estopped = False

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'pick_tomato ready — pick_threshold={self._threshold:.2f}, '
            f'waiting for a tomato.')

    # ── Subscriptions ────────────────────────────────────────────────────────

    def _on_center(self, msg: Point):
        self._center = (msg.x, msg.y, msg.z)
        self._last_seen = time.time()

    def _on_conf(self, msg: Float64):
        self._conf = float(msg.data)

    def _on_estop(self, msg: Bool):
        engaged = bool(msg.data)
        if engaged == self._estopped:
            return
        self._estopped = engaged
        if engaged:
            self.get_logger().warn('E-STOP ENGAGED — halting pick.')
        else:
            # Restart: drop progress, resume hunting from the top. Joint
            # positions are left as-is so resume doesn't snap the arm.
            self._stable = 0
            self._approached = False
            self._enter(SEARCH)
            self.get_logger().warn('E-STOP released — restarting from SEARCH.')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _target_visible(self):
        return (time.time() - self._last_seen) <= self._timeout

    def _confident(self):
        return self._target_visible() and self._conf >= self._threshold

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self.positions)
        self.joint_pub.publish(msg)

    def _set_gripper(self, pct: float):
        m = Float64()
        m.data = float(pct)
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
                    f'Tomato locked (conf={self._conf:.2f}) — centering.')
                self._stable = 0
                self._enter(CENTERING)

        elif st == CENTERING:
            if not self._confident():
                self.get_logger().warn('Target lost while centering — back to search.')
                self._stable = 0
                self._enter(SEARCH)
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
                self.positions[1] += self._appr_sh   # dip shoulder
                self.positions[2] += self._appr_el    # bend elbow
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
                self.positions[1] -= self._lift_sh
                self._publish_joints()
            if self._elapsed() >= self._settle:
                self._enter(DONE)

        elif st == DONE:
            if self._elapsed() < 0.2:
                self.get_logger().info('Tomato picked. Holding position.')

    def _nudge(self, idx: int, error: float):
        step = max(-self._max_step, min(self._max_step, error * self._gain))
        self.positions[idx] += step


def main():
    rclpy.init()
    node = PickTomato()
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
