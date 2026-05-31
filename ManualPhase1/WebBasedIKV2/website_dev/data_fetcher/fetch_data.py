"""
fetch_data.py  —  web ↔ ROS bridge (publisher side)

Listens to the Flask/SocketIO web server and forwards slider values into ROS2:

  /target_pose       geometry_msgs/PoseStamped   position + orientation for IK
  /roboarm/gripper   std_msgs/Float64             gripper open % [0-100]

The IK node (ik_mover) subscribes to /target_pose, plans with MoveIt2, and
executes the trajectory.  bridge_node reads /joint_states and drives Arduino.
"""

import math
import os
import sys
import threading
import time

import base64

import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Image as ImageMsg
from cv_bridge import CvBridge
import socketio

from param import Home_Position, Default_Gripper_Position, Camera_Topics

# Only publish once the user has stopped moving the slider for this long.
# Prevents flooding the IK solver with every pixel of drag.
DEBOUNCE_SEC = 0.20

# Throttle /joint_states → web at ~30 Hz; the URDF viewer doesn't need
# the broadcaster's 100 Hz and SocketIO over a slow link would queue up.
JOINT_STATE_MIN_PERIOD = 1.0 / 30.0

# Throttle camera frames to 15 Hz — base64 JPEG over socket.io is heavy.
CAMERA_MIN_PERIOD = 1.0 / 15.0

# ── Workspace bounds (must match ik_mover.cpp constants) ──────────────────────
_ARM_MAX_REACH = 0.75   # metres
_ARM_MIN_REACH = 0.10   # metres

# Web server (Flask/SocketIO) URL. MUST be the same instance the browser
# connects to, or slider events never reach this node. Defaults to the local
# server; override for remote/ngrok with e.g.
#   WEB_SERVER_URL=https://flying-scorpion-neat.ngrok-free.app
WEB_SERVER_URL = os.environ.get('WEB_SERVER_URL', 'http://localhost:8080')

sio = socketio.Client()
ros_node = None

_publish_timer: threading.Timer | None = None
_timer_lock = threading.Lock()

latest_values = {
    'slider_x':      Home_Position[0],
    'slider_y':      Home_Position[1],
    'slider_z':      Home_Position[2],
    'roll':          Home_Position[3],
    'pitch':         Home_Position[4],
    'yaw':           Home_Position[5],
    'slider_gripper': Default_Gripper_Position,
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _euler_zyx_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """
    ZYX extrinsic Euler angles (degrees) → quaternion (qx, qy, qz, qw).
    Matches the standard used by tf2/KDL.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,   # qx
        cr * sp * cy + sr * cp * sy,   # qy
        cr * cp * sy - sr * sp * cy,   # qz
        cr * cp * cy + sr * sp * sy,   # qw
    )


# ── ROS node ──────────────────────────────────────────────────────────────────

class WebPublisher(Node):
    def __init__(self):
        super().__init__('web_pose_publisher')
        self.pose_pub    = self.create_publisher(PoseStamped, '/target_pose',     10)
        self.gripper_pub = self.create_publisher(Float64,     '/roboarm/gripper', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self._last_js_emit = 0.0
        self._cv_bridge = CvBridge()

        # Build a subscription per configured camera. Each callback is bound
        # to the camera id so the web side can demux frames from any number
        # of feeds onto a single socket.io event.
        self._last_cam_emit: dict[str, float] = {}
        self._camera_subs = []
        for cam in Camera_Topics:
            cam_id = cam['id']
            topic = cam['topic']
            self._last_cam_emit[cam_id] = 0.0
            sub = self.create_subscription(
                ImageMsg, topic,
                lambda msg, cid=cam_id: self._on_camera_frame(cid, msg),
                10)
            self._camera_subs.append(sub)
            self.get_logger().info(
                f"camera feed '{cam_id}' subscribed on {topic}")

    def _on_joint_state(self, msg: JointState):
        now = time.monotonic()
        if now - self._last_js_emit < JOINT_STATE_MIN_PERIOD:
            return
        self._last_js_emit = now
        if not sio.connected:
            return
        try:
            sio.emit('joint_states', {
                'name':     list(msg.name),
                'position': list(msg.position),
            })
        except Exception as e:
            self.get_logger().warn(f'joint_states emit failed: {e}')

    def _on_camera_frame(self, camera_id: str, msg: ImageMsg):
        now = time.monotonic()
        if now - self._last_cam_emit.get(camera_id, 0.0) < CAMERA_MIN_PERIOD:
            return
        self._last_cam_emit[camera_id] = now
        if not sio.connected:
            return
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            b64 = base64.b64encode(buf).decode('utf-8')
            sio.emit('camera_frame', {'camera_id': camera_id, 'frame': b64})
        except Exception as e:
            self.get_logger().warn(
                f"camera_frame emit failed for '{camera_id}': {e}")

    def publish(self, values: dict):
        x = float(values['slider_x'])
        y = float(values['slider_y'])
        z = float(values['slider_z'])

        # ── Workspace guard — hard-block poses that will always fail ──────────
        reach = math.sqrt(x*x + y*y + z*z)
        if reach < _ARM_MIN_REACH:
            self.get_logger().warn(
                f'Target reach {reach:.3f} m < min {_ARM_MIN_REACH} m '
                f'(pos={x:.3f},{y:.3f},{z:.3f}) — skipping publish.')
            return
        if reach > _ARM_MAX_REACH:
            self.get_logger().warn(
                f'Reach {reach:.3f} m > max {_ARM_MAX_REACH} m — '
                f'ik_mover will stretch-clamp toward target.')

        # ── Orientation (slider gives degrees, IK needs quaternion) ───────────
        qx, qy, qz, qw = _euler_zyx_to_quaternion(
            float(values['roll']),
            float(values['pitch']),
            float(values['yaw']),
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x    = x
        pose_msg.pose.position.y    = y
        pose_msg.pose.position.z    = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        # ── Gripper (independent of IK) ────────────────────────────────────────
        g = max(0.0, min(100.0, float(values['slider_gripper'])))
        gripper_msg = Float64()
        gripper_msg.data = g
        self.gripper_pub.publish(gripper_msg)

        self.get_logger().info(
            f'/target_pose pos=({x:.3f},{y:.3f},{z:.3f})  '
            f'q=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})  '
            f'gripper={g:.1f}%'
        )


# ── SocketIO callbacks ────────────────────────────────────────────────────────

@sio.event
def connect():
    print('Connected to web server')


@sio.event
def connect_error(error):
    print(f'Connection failed: {error}')


@sio.event
def disconnect():
    print('Disconnected from web server')


def _fire_publish():
    """Called by the debounce timer — publishes the latest snapshot."""
    if ros_node is not None:
        ros_node.publish(latest_values)


@sio.on('value_updated')
def on_value_updated(data):
    global _publish_timer
    latest_values[data['param']] = data['value']   # always update dict immediately

    # Reset the debounce timer so we only publish after the user stops dragging
    with _timer_lock:
        if _publish_timer is not None:
            _publish_timer.cancel()
        _publish_timer = threading.Timer(DEBOUNCE_SEC, _fire_publish)
        _publish_timer.daemon = True
        _publish_timer.start()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global ros_node

    rclpy.init()
    ros_node = WebPublisher()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Only publish home position if it is within the reachable workspace.
    # Home_Position defaults to [0,0,0,...] which is the robot's base origin —
    # publishing that would immediately trigger an IK failure loop.
    home_reach = math.sqrt(
        Home_Position[0]**2 + Home_Position[1]**2 + Home_Position[2]**2)
    if home_reach >= _ARM_MIN_REACH:
        ros_node.publish(latest_values)
        print(f'Home position published: {latest_values}')
    else:
        print(
            f'Home position {Home_Position[:3]} is at the origin — '
            f'skipping initial publish. Arm will move on first slider input.')

    try:
        print(f'Connecting to web server at {WEB_SERVER_URL} …')
        sio.connect(WEB_SERVER_URL, wait_timeout=10)
        sio.wait()
    except Exception as e:
        print(f'Error: {e}')
        print(f'Ensure the Flask server (app.py) is running and reachable at {WEB_SERVER_URL}')
        sys.exit(1)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
