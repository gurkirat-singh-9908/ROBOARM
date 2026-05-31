from flask import Flask, render_template, request, jsonify, send_from_directory, Response
from flask_talisman import Talisman
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
import os
import re
from flask_socketio import SocketIO, emit
from data_fetcher import param

# ── URDF / mesh paths (served live from the ROS2 description package) ─────────
_ROS_DESC_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'ROBO_ws', 'src', 'roboticarm_description'
))
_XACRO_PATH = os.path.join(_ROS_DESC_DIR, 'urdf', 'roboticarm.xacro')
_MESHES_DIR = os.path.join(_ROS_DESC_DIR, 'meshes')

def _load_urdf_for_web():
    """
    Read the xacro and adapt it for the browser:
      - drop <xacro:include> directives (materials/transmissions/gazebo
        aren't needed by the URDF viewer)
      - rewrite mesh paths from file://$(find ...)/meshes/ → meshes/
        (relative — urdf-loader resolves them against the URDF's
        workingPath of '/', giving '/meshes/<file>'. Using '/meshes/'
        directly would yield '//meshes/<file>' which the browser
        treats as a protocol-relative URL to a host named 'meshes'.)
    """
    with open(_XACRO_PATH, 'r') as f:
        text = f.read()
    text = re.sub(r'<xacro:include\b[^>]*?/>\s*', '', text)
    text = re.sub(
        r'file://\$\(find roboticarm_description\)/meshes/',
        'meshes/',
        text,
    )
    return text

_ROBOT_URDF = _load_urdf_for_web()

app = Flask(__name__)
app.config['SECRET_KEY'] = os.urandom(24)  # Generate a secure secret key

# Security headers (force_https=False for development without SSL certs)
Talisman(app,
    force_https=False,
    content_security_policy={
        'default-src': "'self'",
        'script-src': ["'self'", "'unsafe-inline'", 'cdn.jsdelivr.net'],
        'style-src': ["'self'", "'unsafe-inline'", 'cdn.jsdelivr.net'],
        'img-src': ["'self'", 'data:'],
    }
)

# Rate limiting
limiter = Limiter(
    app=app,
    key_func=get_remote_address,
    default_limits=["200 per day", "50 per hour"]
)

# Initialize SocketIO
socketio = SocketIO(app, async_mode="threading")

# Error handlers
@app.errorhandler(404)
def page_not_found(e):
    return render_template('errors/404.html'), 404

@app.errorhandler(500)
def internal_server_error(e):
    return render_template('errors/500.html'), 500

# Routes
@app.route("/")
def HOME():
    return render_template('index.html')

@app.route("/MANUAL_CONTROL")
def MANUAL_CONTROL():
    return render_template('MANUAL_CONTROL.html')

@app.route("/AUTOMATIC_CONTROL")
def AUTOMATIC_CONTROL():
    return render_template('AUTOMATIC_CONTROL.html')

@app.route("/Connect")
def Connect():
    return render_template('Connect.html')

@app.route("/api/cameras")
def list_cameras():
    return jsonify([
        {'id': c['id'], 'name': c.get('name', c['id'])}
        for c in param.Camera_Topics
    ])

@app.route("/robot_description")
def robot_description():
    return Response(_ROBOT_URDF, mimetype='application/xml')

@app.route("/meshes/<path:filename>")
def serve_mesh(filename):
    return send_from_directory(_MESHES_DIR, filename)

# API endpoints for robotic arm control
@app.route("/api/position", methods=['POST'])
@limiter.limit("100 per minute")
def update_position():
    data = request.get_json()
    return jsonify({"status": "success"})

@app.route("/api/orientation", methods=['POST'])
@limiter.limit("100 per minute")
def update_orientation():
    data = request.get_json()
    return jsonify({"status": "success"})

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('update_value')
def handle_update_value(data):
    print(f"Received data: {data}")
    # Broadcast the updated value to all connected clients
    emit('value_updated', data, broadcast=True)

@socketio.on('joint_states')
def handle_joint_states(data):
    # Forwarded by data_fetcher/fetch_data.py from ROS /joint_states.
    # Re-broadcast to browser clients only (skip the publisher itself).
    emit('joint_states', data, broadcast=True, include_self=False)

@socketio.on('camera_frame')
def handle_camera_frame(data):
    # Forwarded by data_fetcher/fetch_data.py from ROS /camera/image_raw.
    # Re-broadcast to browser clients only (skip the publisher itself).
    emit('camera_frame', data, broadcast=True, include_self=False)

if __name__ == "__main__":
    socketio.run(app,host="0.0.0.0",port=8080,allow_unsafe_werkzeug=True)
