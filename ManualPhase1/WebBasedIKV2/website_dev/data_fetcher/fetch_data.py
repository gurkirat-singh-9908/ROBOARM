import socketio
import time
import sys
import serial
import threading
#from main import inverse_kinematics
from map import map_sliders_to_servos
import numpy as np
from param import Home_Position, Default_Gripper_Position

port = '/dev/ttyUSB0'  # Change to your port (e.g., 'COM6' or '/dev/ttyUSB0')
baud_rate = 115200
# Create a Socket.IO client
sio = socketio.Client()
arduino_flag = True

# Emergency stop flag
emergency_stopped = False

# Packet sequence number (0-255 rolling)
packet_seq = 0

# Threading lock for latest_values
state_lock = threading.Lock()

# Store the latest values
#TODO: Wrong data name and type used for angles. it should be distnce for x, y and z but its angles in gripper is should be percentage val but its again angle
latest_values = {
    'slider_x': Home_Position[0],
    'slider_y': Home_Position[1],
    'slider_z': Home_Position[2],
    'roll': Home_Position[3],
    'pitch': Home_Position[4],
    'yaw': Home_Position[5],
    'slider_gripper': Default_Gripper_Position
}

#TODO: hard coding the gripper value for now.
def send_data_to_arduino(servo_angles):
    """
    Sends servo_angles to Arduino with ARM header, sequence number, and checksum.

    Args:
        servo_angles: all the angles
    """
    global packet_seq
    if emergency_stopped:
        return
    if not arduino_serial or not arduino_serial.is_open:
        print("Arduino not connected — skipping send")
        return
    s1 = int(round(servo_angles["s1"]))
    s2 = int(round(servo_angles["s2"]))
    s3 = int(round(servo_angles["s3"]))
    s4 = int(round(servo_angles["s4"]))
    s5 = int(round(servo_angles["s5"]))
    s6 = int(round(servo_angles["s6"]))
    g  = int(round(servo_angles.get("gripper", 50)))  # keep gripper at 50 for now
    seq = packet_seq
    packet_seq = (packet_seq + 1) % 256
    checksum = (s1 + s2 + s3 + s4 + s5 + s6 + g + seq) % 256
    command = f"ARM {s1} {s2} {s3} {s4} {s5} {s6} {g} {seq} {checksum}\n"
    arduino_serial.write(command.encode())
    print(f"TX [{seq}]: {command.strip()}")

# Serial connection to Arduino
arduino_serial = None

def connect_to_arduino():
    """Connect to Arduino on COM6."""
    global arduino_serial, port
    max_connect_attempts = 3
    for attempt in range(max_connect_attempts):
        try:
            arduino_serial = serial.Serial(port, 115200, timeout=0.2)
            print(f"Connected to Arduino on {port}")
            time.sleep(2)  # Wait for Arduino reset
            #arduino_serial.flushInput()  # Clear input buffer
            #arduino_serial.flushOutput()  # Clear output buffer
            return True
        except (OSError, serial.SerialException) as e:
            print(f"Failed to connect on attempt {attempt + 1}/{max_connect_attempts}: {e}")
            if arduino_serial:
                arduino_serial.close()
                arduino_serial = None
            time.sleep(1)
    print("Warning: Could not connect to Arduino. Running without hardware control.")
    return False

def map_sliders_to_matrix(values):
    """
    Maps all slider values to their corresponding servo angles

    Args:
        values (dict): Dictionary containing slider values

    Returns:
        dict: Dictionary containing mapped servo angles
    """

    # Convert string values to float before mapping
    #try:
    float_values = {
        'slider_x': float(values['slider_x']),
        'slider_y': float(values['slider_y']),
        'slider_z': float(values['slider_z']),
        'roll': float(values['roll']),
        'pitch': float(values['pitch']),
        'yaw': float(values['yaw']),
        'slider_gripper': float(values['slider_gripper'])
    }

    # Map each slider to its corresponding servo
    desiredMatrix = np.array([[float_values['slider_x'], float_values['slider_y'], float_values['slider_z']],
                    [float_values['roll'], float_values['pitch'], float_values['yaw']]])

    return desiredMatrix

    '''except (ValueError, TypeError) as e:
        print(f"Error converting values to float: {e}")
        print("Values received:", values)
        return None'''

@sio.event
def connect():
    print('Connected to server')

@sio.event
def connect_error(error):
    print(f'Connection failed: {error}')

@sio.event
def disconnect():
    print('Disconnected from server')

@sio.on('emergency_stop')
def on_emergency_stop():
    global emergency_stopped
    emergency_stopped = True
    print("EMERGENCY STOP active — all commands blocked")

@sio.on('value_updated')
def on_value_updated(data):
    global arduino_flag
    param = data.get('param')
    value = data.get('value')
    with state_lock:
        latest_values[param] = value
        current_values = dict(latest_values)

    # Map slider values to servo angles
    desiredMatrix = map_sliders_to_matrix(current_values)
    print(">> desiredMatrix =", desiredMatrix)
    #servo_angles = inverse_kinematics(desiredMatrix)
    servo_angles = map_sliders_to_servos(current_values)


    if arduino_flag:
        send_data_to_arduino(servo_angles)
    else:
        print(servo_angles)

@sio.on('position_set')
def on_position_set(data):
    global arduino_flag
    with state_lock:
        latest_values.update(data)
        current_values = dict(latest_values)
    servo_angles = map_sliders_to_servos(current_values)
    if servo_angles and arduino_flag:
        send_data_to_arduino(servo_angles)

def main():
    try:
        # Connect to Arduino
        global arduino_flag
        arduino_connected = connect_to_arduino()
        if arduino_connected:
            arduino_flag = True
            print("Sending initial servo positions...")
            print(f"latest_values{latest_values}")
            desiredMatrix = map_sliders_to_matrix(latest_values)
            print(">> desiredMatrix =", desiredMatrix)
            #servo_angles = inverse_kinematics(desiredMatrix)
            servo_angles = map_sliders_to_servos(latest_values)
            send_data_to_arduino(servo_angles)

        else:
            print("Warning: Could not connect to Arduino. Running without hardware control.")
            arduino_flag = False

        while True:
            try:
                print("Connecting to server at http://localhost:8080 ...")
                sio.connect('http://localhost:8080', wait_timeout=10)
                sio.wait()
            except Exception as e:
                print(f"Connection lost: {e}. Retrying in 5 seconds...")
                time.sleep(5)
            finally:
                try:
                    sio.disconnect()
                except:
                    pass
    except Exception as e:
        print(f'Error: {e}')
        print('Ensure Flask server is running on http://localhost:8080')
        if arduino_serial and arduino_serial.is_open:
            arduino_serial.close()
            print("Arduino connection closed")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    finally:
        pass
