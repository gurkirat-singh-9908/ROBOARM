from flask import Flask, render_template, request, jsonify
from flask_talisman import Talisman
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
import os
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = os.urandom(24)  # Generate a secure secret key

# Security headers
Talisman(app, 
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
socketio = SocketIO(app)

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

@app.route("/Model")
def Model():
    return render_template('index2.html')

# API endpoints for robotic arm control
@app.route("/api/position", methods=['POST'])
@limiter.limit("100 per minute")
def update_position():
    data = request.get_json()
    # TODO: Implement position update logic
    return jsonify({"status": "success"})

@app.route("/api/orientation", methods=['POST'])
@limiter.limit("100 per minute")
def update_orientation():
    data = request.get_json()
    # TODO: Implement orientation update logic
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

if __name__ == "__main__":
    socketio.run(app, debug=True, port=8080)
