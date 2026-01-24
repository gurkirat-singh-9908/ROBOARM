from serial import Serial
import time
import tkinter as tk

slider_val = [90, 90, 90, 90, 90, 90]

def slider_value(value, index):
    """
    Update the slider value when a slider is moved.
    """
    global slider_val
    slider_val[index] = int(value)

def send_serial_data():
    """
    Send the current slider values to the Arduino via serial.
    """
    global slider_val
    # Convert the list to a comma-separated string
    data_str = ",".join(map(str, slider_val))
    ser.write(data_str.encode())  # Encode string as bytes
    print(f"Sent: {data_str}")
    # Schedule the function to run again after 100ms
    root.after(100, send_serial_data)

# Initialize serial connection
ser = Serial('/dev/ttyUSB0', 9600)  
time.sleep(2)  # Wait for Arduino to initialize

# Create the main window
root = tk.Tk()
root.title("Six Sliders with Default Values")

# Create 6 sliders dynamically
sliders = []

for i in range(6):
    tk.Label(root, text=f"Slider {i + 1}").pack()
    slider = tk.Scale(
        root, 
        from_=0, 
        to=180, 
        orient='horizontal', 
        command=lambda value, idx=i: slider_value(value, idx)
    )
    slider.pack()
    slider.set(90)  # Set default value
    sliders.append(slider)

# Start sending serial data
root.after(1, send_serial_data)

# Start the Tkinter main loop
root.mainloop()

# Close the serial connection after exiting the GUI
ser.close()
