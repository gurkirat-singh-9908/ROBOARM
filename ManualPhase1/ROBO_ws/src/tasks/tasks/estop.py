"""
estop.py  —  fire the arm kill switch from the command line

Publishes std_msgs/Bool on /roboarm/estop, the topic the arduino_bridge and
the pick task both watch:

  True  = KILL   — bridge freezes (drops all serial writes, servos hold),
                   pick halts its state machine.
  False = START  — bridge resumes, pick restarts from SEARCH.

Usage:
  ros2 run tasks estop stop      # or: kill | true | 1   → engage
  ros2 run tasks estop start     # or: run  | false | 0  → release

The node publishes a few times over ~0.5 s (so every running subscriber
gets it) and then exits. Run it while the bridge / picker are up.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

_ENGAGE = {'stop', 'kill', 'estop', 'halt', 'true', '1', 'on'}
_RELEASE = {'start', 'run', 'resume', 'go', 'false', '0', 'off'}


class EStopPublisher(Node):

    def __init__(self, engage: bool):
        super().__init__('estop_trigger')
        self.pub = self.create_publisher(Bool, '/roboarm/estop', 10)
        self._engage = engage

    def fire(self):
        msg = Bool()
        msg.data = self._engage
        # Repeat briefly so late-matched subscribers still receive it.
        deadline = time.time() + 0.5
        while time.time() < deadline:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().info(
            f"/roboarm/estop = {self._engage}  "
            f"({'KILL' if self._engage else 'START'})")


def main():
    arg = (sys.argv[1] if len(sys.argv) > 1 else '').strip().lower()
    if arg in _ENGAGE:
        engage = True
    elif arg in _RELEASE:
        engage = False
    else:
        print('usage: ros2 run tasks estop <stop|start>', file=sys.stderr)
        print(f'  engage:  {sorted(_ENGAGE)}', file=sys.stderr)
        print(f'  release: {sorted(_RELEASE)}', file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = EStopPublisher(engage)
    try:
        node.fire()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
