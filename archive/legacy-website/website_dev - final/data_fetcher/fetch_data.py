import socketio
import time
import sys

# Create a Socket.IO client
sio = socketio.Client()

# Store the latest values
latest_values = {
    'slider_x': 0,
    'slider_y': 0,
    'slider_z': 0,
    'roll': 0,
    'pitch': 0,
    'yaw': 0,
    'slider_gripper': 0
}

@sio.event
def connect():
    print('Connected to server')

@sio.event
def connect_error(error):
    print(f'Connection failed: {error}')

@sio.event
def disconnect():
    print('Disconnected from server')

@sio.on('value_updated')
def on_value_updated(data):
    param = data.get('param')
    value = data.get('value')
    latest_values[param] = value
    print('\nCurrent Values:')
    print('-' * 20)
    print(f'Position:')
    print(f'  X: {latest_values["slider_x"]}')
    print(f'  Y: {latest_values["slider_y"]}')
    print(f'  Z: {latest_values["slider_z"]}')
    print(f'Orientation:')
    print(f'  Roll: {latest_values["roll"]}°')
    print(f'  Pitch: {latest_values["pitch"]}°')
    print(f'  Yaw: {latest_values["yaw"]}°')
    print(f'Gripper:')
    print(f'  Opening: {latest_values["slider_gripper"]}%')

def main():
    try:
        print('Connecting to server...')
        sio.connect('http://localhost:8080', wait_timeout=10)
        print('Connected! Waiting for updates...')
        sio.wait()
    except Exception as e:
        print(f'Error: {e}')
        print('Make sure the Flask server is running on http://localhost:8080')
        sys.exit(1)

if __name__ == '__main__':
    main() 