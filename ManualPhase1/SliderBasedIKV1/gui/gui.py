import tkinter as tk
from tkinter import ttk
import serial
import time

#chek

# Initialize serial communication
try:
    arduino = serial.Serial(port="COM6", baudrate=9600, timeout=1)
except Exception as e:
    print(f"Error: {e}")
    arduino = None

# Function to send data to Arduino
def send_data():
    if arduino:
        packet = "<{S["
        for i in range(6):
            packet += str(sliders[i].get())
            if i < 5:
                packet += ","
        packet += "]}>"
        arduino.write((packet + "\n").encode())
        print(f"Packet sent: {packet}")

        # Read response from Arduino
        response = arduino.readline().decode().strip()
        if response:
            print(f"Response: {response}")

# Create GUI
root = tk.Tk()
root.title("Servo Controller")

# Create sliders for servo control
sliders = []
for i in range(6):
    frame = ttk.Frame(root)
    frame.pack(pady=10)

    label = ttk.Label(frame, text=f"Servo {i + 1}:")
    label.pack(side=tk.LEFT, padx=5)

    slider = ttk.Scale(frame, from_=0, to=180, orient=tk.HORIZONTAL, length=300)
    slider.set(90)  # Default position
    slider.pack(side=tk.LEFT, padx=5)

    sliders.append(slider)

# Send button to send data to Arduino
send_button = ttk.Button(root, text="Send", command=send_data)
send_button.pack(pady=20)

# Run the GUI event loop
root.mainloop()

# Close the serial port when the program exits
if arduino:
    arduino.close()
