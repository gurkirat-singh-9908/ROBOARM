from flask import Flask, render_template, request, jsonify
from flask_talisman import Talisman
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
import os
import json
from datetime import datetime, timezone
from pathlib import Path
from flask_socketio import SocketIO, emit, join_room
import param

app = Flask(__name__)
app.config['SECRET_KEY'] = os.getenv('SECRET_KEY', 'dev-secret-change-me')

BASE_DIR = Path(__file__).resolve().parent
FEEDBACK_FILE = BASE_DIR / 'feedback_submissions.jsonl'

# Security headers
Talisman(app, 
    content_security_policy={
        'default-src': "'self'",
        'script-src': ["'self'", 'cdn.jsdelivr.net'],
        'style-src': ["'self'", 'cdn.jsdelivr.net', 'fonts.googleapis.com'],
        'font-src': ["'self'", 'fonts.gstatic.com', 'cdn.jsdelivr.net'],
        'img-src': ["'self'", 'data:'],
        'connect-src': ["'self'", 'ws:', 'wss:'],
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


def _to_float_in_range(payload, key, minimum, maximum):
    value = payload.get(key)
    if value is None:
        raise ValueError(f"Missing field: {key}")
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"Invalid number for {key}")
    if not minimum <= value <= maximum:
        raise ValueError(f"{key} must be between {minimum} and {maximum}")
    return value


def _write_feedback(payload):
    FEEDBACK_FILE.parent.mkdir(parents=True, exist_ok=True)
    with FEEDBACK_FILE.open('a', encoding='utf-8') as f:
        f.write(json.dumps(payload, ensure_ascii=False) + '\n')

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

# API endpoints for robotic arm control
@app.route("/api/position", methods=['POST'])
@limiter.limit("100 per minute")
def update_position():
    data = request.get_json(silent=True) or {}
    try:
        x = _to_float_in_range(data, 'x', -0.8, 0.8)
        y = _to_float_in_range(data, 'y', -0.8, 0.8)
        z = _to_float_in_range(data, 'z', -0.8, 0.8)
    except ValueError as exc:
        return jsonify({'status': 'error', 'message': str(exc)}), 400

    return jsonify({"status": "success", "position": {'x': x, 'y': y, 'z': z}})

@app.route("/api/orientation", methods=['POST'])
@limiter.limit("100 per minute")
def update_orientation():
    data = request.get_json(silent=True) or {}
    try:
        roll = _to_float_in_range(data, 'roll', 0, 180)
        pitch = _to_float_in_range(data, 'pitch', 0, 180)
        yaw = _to_float_in_range(data, 'yaw', 0, 180)
    except ValueError as exc:
        return jsonify({'status': 'error', 'message': str(exc)}), 400

    return jsonify({"status": "success", "orientation": {'roll': roll, 'pitch': pitch, 'yaw': yaw}})


@app.route('/api/feedback', methods=['POST'])
@limiter.limit('5 per hour')
def submit_feedback():
    data = request.get_json(silent=True) or {}
    rating = data.get('rating')
    comment = str(data.get('comment', '')).strip()
    page = str(data.get('page', '')).strip()[:120]

    try:
        rating = int(rating)
    except (TypeError, ValueError):
        return jsonify({'status': 'error', 'message': 'rating must be an integer'}), 400

    if rating < 1 or rating > 5:
        return jsonify({'status': 'error', 'message': 'rating must be between 1 and 5'}), 400
    if len(comment) > 500:
        return jsonify({'status': 'error', 'message': 'comment cannot exceed 500 characters'}), 400

    payload = {
        'rating': rating,
        'comment': comment,
        'page': page or request.path,
        'submitted_at': datetime.now(timezone.utc).isoformat(),
        'remote_addr': request.remote_addr,
    }
    _write_feedback(payload)
    return jsonify({'status': 'success'})

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')


@socketio.on('join_channel')
def handle_join_channel(data):
    channel = (data or {}).get('channel')
    if not channel:
        emit('control_error', {'message': 'Missing channel'})
        return
    join_room(channel)
    emit('channel_joined', {'channel': channel})


@socketio.on('update_value')
def handle_update_value(data):
    if not isinstance(data, dict):
        emit('control_error', {'message': 'Invalid payload'})
        return
    channel = data.get('channel')
    if not channel:
        emit('control_error', {'message': 'Missing channel'})
        return
    print(f"Received data: {data}")
    emit('value_updated', data, room=channel)

if __name__ == "__main__":
    socketio.run(app,host="0.0.0.0",port=8080,allow_unsafe_werkzeug=True)
