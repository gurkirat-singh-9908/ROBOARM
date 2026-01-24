import serial
import time

def send_data_to_arduino(ser, value1, value2):
    """
    Sends two integers to Arduino with a checksum and prints the response.
    
    Args:
        ser: Serial connection object
        value1: First integer to send
        value2: Second integer to send
    """
    checksum = value1 + value2
    command = f"{value1} {value2} {checksum}\n"
    ser.write(command.encode())  # Send the command as bytes
    time.sleep(0.05)  # Brief delay for Arduino to respond
    response = ser.readline().decode().strip()  # Read and decode response
    print(f"Sent: {command.strip()}, Received: {response}")

# Main execution
port = 'COM6'  # Change to your port (e.g., 'COM6' or '/dev/ttyUSB0')
baud_rate = 9600

try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    time.sleep(2)  # Wait for Arduino to initialize
    # Loop to send data 10 times
    for i in range(100):
        send_data_to_arduino(ser, i+1, i*15)  # Send i and i+1
except serial.SerialException as e:
    print(f"Serial error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed")