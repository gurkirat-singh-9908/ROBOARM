import socketio
import time
import sys
import serial
#from main import inverse_kinematics
from map import map_sliders_to_servos
import numpy as np
from param import Home_Position, Default_Gripper_Position

port = '/dev/ttyUSB0'  # Change to your port (e.g., 'COM6' or '/dev/ttyUSB0')
baud_rate = 9600
# Create a Socket.IO client
sio = socketio.Client()
arduino_flag = True

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
    Sends servo_angles to Arduino with a checksum and prints the response.
    
    Args:
        ser: Serial connection object
        servo_angles: all the angels 
    """
    print(f"send data to arduino executed")
    s1,s2,s3,s4,s5,s6,g = servo_angles["s1"], servo_angles["s2"], servo_angles["s3"], servo_angles["s4"], servo_angles["s5"], servo_angles["s6"], 50
    checksum = s1+s2+s3+s4+s5+s6+g
    command = f"{s1} {s2} {s3} {s4} {s5} {s6} {g} {checksum}\n"
    arduino_serial.write(command.encode())  # Send the command as bytes
    time.sleep(0.05)  # Brief delay for Arduino to respond
    response = arduino_serial.readline().decode().strip()  # Read and decode response
    print(f"Sent: {command.strip()}, Received: {response}")

# Serial connection to Arduino
arduino_serial = None

def connect_to_arduino():
    """Connect to Arduino on COM6."""
    global arduino_serial, port
    max_connect_attempts = 3
    for attempt in range(max_connect_attempts):
        try:
            arduino_serial = serial.Serial(port, 9600, timeout=0.2)
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

@sio.on('value_updated')
def on_value_updated(data):
    global arduino_flag
    param = data.get('param')
    value = data.get('value')
    latest_values[param] = value
    
    # Map slider values to servo angles
    desiredMatrix = map_sliders_to_matrix(latest_values)
    print(">> desiredMatrix =", desiredMatrix)
    #servo_angles = inverse_kinematics(desiredMatrix)
    servo_angles = map_sliders_to_servos(latest_values)

    
    if arduino_flag:
        send_data_to_arduino(servo_number, intensity)
    else:
        print(servo_number, intensity)

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
        
        print('Connecting to server...')
        sio.connect('http://localhost:8080', wait_timeout=10)
        sio.wait()
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
